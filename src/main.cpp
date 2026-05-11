#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include <math.h>

namespace {

constexpr const char *AP_SSID = "mppt";
constexpr const char *AP_PASSWORD = "password";
constexpr const char *HOSTNAME = "mppt";

constexpr uint32_t SERIAL_BAUD = 9600;
constexpr uint32_t FRAME_GAP_MS = 30;
constexpr size_t MAX_FRAME = 96;
constexpr float PV_CURRENT_LOSS_FACTOR = 1.0526f;
constexpr float DARK_PV_WATTS_THRESHOLD = 5.0f;
constexpr float DAY_START_PV_WATTS_THRESHOLD = 200.0f;
constexpr uint32_t NEW_DAY_DARK_MS = 60UL * 60UL * 1000UL;
constexpr uint32_t MAX_DAILY_INTEGRATION_GAP_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t INITIAL_TOTAL_ENERGY_KWH = 0;
constexpr uint32_t MAX_REASONABLE_TOTAL_ENERGY_KWH = 9999999;

// Seeed XIAO ESP32-S3 pin map:
// D7 = GPIO44, labeled RX. Use it for the controller TX/value direction.
// D1 = GPIO2. Use it for the display TX/request direction.
constexpr int RX_CONTROLLER_PIN = 44;
constexpr int RX_DISPLAY_PIN = 2;

constexpr uint32_t STA_RETRY_WINDOW_MS = 2UL * 60UL * 1000UL;
constexpr uint32_t AP_RECONFIG_WINDOW_MS = 5UL * 60UL * 1000UL;
constexpr uint32_t WIFI_POLL_MS = 1000;

HardwareSerial controllerSerial(1);
HardwareSerial displaySerial(2);
WebServer server(80);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
Preferences prefs;

enum class WifiPhase {
  SetupAp,
  StaRetry,
  StaConnected,
  ApReconfig,
};

struct FrameBuffer {
  const char *name;
  HardwareSerial *serial;
  uint8_t data[MAX_FRAME];
  size_t length = 0;
  uint32_t firstByteMs = 0;
  uint32_t lastByteMs = 0;
  uint32_t frames = 0;
  uint32_t crcErrors = 0;
  String lastFrameHex;
  bool lastFrameCrcOk = false;
};

struct Metrics {
  float batteryVolts = NAN;
  float pvVolts = NAN;
  float pvAmps = NAN;
  float pvWatts = NAN;
  float estimatedBatteryAmps = NAN;
  float batterySocPercent = NAN;
  float dailyWh = NAN;
  float totalKWh = NAN;
  uint16_t chargeStateRaw = 0;
  uint16_t pvWattsRaw = 0;
  bool chargeActive = false;
  int deviceTempC = INT_MIN;
  uint16_t loadFlags = 0;
  uint32_t lastUpdateMs = 0;
  uint32_t decodedFrames = 0;
};

FrameBuffer controllerRx{"controller", &controllerSerial};
FrameBuffer displayRx{"display", &displaySerial};
Metrics metrics;

WifiPhase wifiPhase = WifiPhase::SetupAp;
uint32_t wifiPhaseStartedMs = 0;
uint32_t lastWifiPollMs = 0;
bool hasCredentials = false;
String savedSsid;
String savedPassword;
bool apRunning = false;
bool mdnsStarted = false;
bool hasDailyEnergySample = false;
bool dailyResetArmed = false;
uint32_t lastDailyEnergySampleMs = 0;
uint32_t darkStartedMs = 0;
float lastDailyEnergyPvWatts = 0.0f;
uint32_t storedTotalEnergyKWh = 0;

String telnetLine;

uint16_t crc16Modbus(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool frameCrcOk(const uint8_t *data, size_t length) {
  if (length < 4) {
    return false;
  }
  const uint16_t expected = data[length - 2] | (static_cast<uint16_t>(data[length - 1]) << 8);
  return crc16Modbus(data, length - 2) == expected;
}

String hexFrame(const uint8_t *data, size_t length) {
  static const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(length * 3);
  for (size_t i = 0; i < length; ++i) {
    if (i) {
      out += ' ';
    }
    out += hex[data[i] >> 4];
    out += hex[data[i] & 0x0F];
  }
  return out;
}

String jsonEscape(const String &value) {
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (static_cast<uint8_t>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

void appendJsonFloat(String &out, const char *key, float value, uint8_t decimals = 1) {
  out += '"';
  out += key;
  out += "\":";
  if (isnan(value)) {
    out += "null";
  } else {
    out += String(value, static_cast<unsigned int>(decimals));
  }
}

void appendJsonInt(String &out, const char *key, int value) {
  out += '"';
  out += key;
  out += "\":";
  if (value == INT_MIN) {
    out += "null";
  } else {
    out += value;
  }
}

String wifiPhaseName() {
  switch (wifiPhase) {
    case WifiPhase::SetupAp:
      return "setup-ap";
    case WifiPhase::StaRetry:
      return "sta-retry";
    case WifiPhase::StaConnected:
      return "sta-connected";
    case WifiPhase::ApReconfig:
      return "ap-reconfig";
  }
  return "unknown";
}

void telnetPrintln(const String &line) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(line);
  }
}

void loadCredentials() {
  prefs.begin("mppt", false);
  savedSsid = prefs.getString("ssid", "");
  savedPassword = prefs.getString("pass", "");
  hasCredentials = savedSsid.length() > 0;
}

void loadEnergyState() {
  if (prefs.isKey("totalKWh")) {
    storedTotalEnergyKWh = prefs.getUInt("totalKWh", INITIAL_TOTAL_ENERGY_KWH);
    if (storedTotalEnergyKWh > MAX_REASONABLE_TOTAL_ENERGY_KWH) {
      storedTotalEnergyKWh = INITIAL_TOTAL_ENERGY_KWH;
      prefs.putUInt("totalKWh", storedTotalEnergyKWh);
    }
    metrics.totalKWh = static_cast<float>(storedTotalEnergyKWh);
    return;
  }

  storedTotalEnergyKWh = INITIAL_TOTAL_ENERGY_KWH;
  metrics.totalKWh = static_cast<float>(storedTotalEnergyKWh);
  prefs.putUInt("totalKWh", storedTotalEnergyKWh);
}

void saveCredentials(const String &ssid, const String &password) {
  savedSsid = ssid;
  savedPassword = password;
  hasCredentials = savedSsid.length() > 0;
  prefs.putString("ssid", savedSsid);
  prefs.putString("pass", savedPassword);
}

void clearCredentials() {
  savedSsid = "";
  savedPassword = "";
  hasCredentials = false;
  prefs.remove("ssid");
  prefs.remove("pass");
}

void startMdnsIfNeeded() {
  if (!mdnsStarted && WiFi.status() == WL_CONNECTED) {
    mdnsStarted = MDNS.begin(HOSTNAME);
    if (mdnsStarted) {
      MDNS.addService("http", "tcp", 80);
      MDNS.addService("telnet", "tcp", 23);
    }
  }
}

void stopAp() {
  if (apRunning) {
    WiFi.softAPdisconnect(true);
    apRunning = false;
  }
}

void startAp(bool withSta) {
  if (withSta) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }

  IPAddress apIp(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apRunning = true;
}

void beginSta() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
}

void enterSetupAp() {
  wifiPhase = WifiPhase::SetupAp;
  wifiPhaseStartedMs = millis();
  startAp(false);
}

void enterStaRetry() {
  if (!hasCredentials) {
    enterSetupAp();
    return;
  }
  wifiPhase = WifiPhase::StaRetry;
  wifiPhaseStartedMs = millis();
  mdnsStarted = false;
  stopAp();
  beginSta();
}

void enterApReconfig() {
  wifiPhase = WifiPhase::ApReconfig;
  wifiPhaseStartedMs = millis();
  startAp(true);
  if (hasCredentials) {
    WiFi.begin(savedSsid.c_str(), savedPassword.c_str());
  }
}

void enterStaConnected() {
  wifiPhase = WifiPhase::StaConnected;
  wifiPhaseStartedMs = millis();
  stopAp();
  startMdnsIfNeeded();
}

void pollWifiState() {
  const uint32_t now = millis();
  if (now - lastWifiPollMs < WIFI_POLL_MS) {
    return;
  }
  lastWifiPollMs = now;

  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected) {
    if (wifiPhase != WifiPhase::StaConnected) {
      enterStaConnected();
    } else {
      startMdnsIfNeeded();
    }
    return;
  }

  if (wifiPhase == WifiPhase::StaConnected) {
    enterStaRetry();
    return;
  }

  if (wifiPhase == WifiPhase::StaRetry && now - wifiPhaseStartedMs >= STA_RETRY_WINDOW_MS) {
    enterApReconfig();
    return;
  }

  if (wifiPhase == WifiPhase::ApReconfig && now - wifiPhaseStartedMs >= AP_RECONFIG_WINDOW_MS) {
    enterStaRetry();
  }
}

void updateDailyEnergyEstimate(float pvWatts, uint32_t now) {
  if (!isfinite(pvWatts) || pvWatts < 0.0f) {
    pvWatts = 0.0f;
  }
  if (isnan(metrics.dailyWh)) {
    metrics.dailyWh = 0.0f;
  }
  if (!isfinite(metrics.totalKWh) || metrics.totalKWh < 0.0f ||
      metrics.totalKWh > static_cast<float>(MAX_REASONABLE_TOTAL_ENERGY_KWH)) {
    storedTotalEnergyKWh = INITIAL_TOTAL_ENERGY_KWH;
    metrics.totalKWh = static_cast<float>(storedTotalEnergyKWh);
    prefs.putUInt("totalKWh", storedTotalEnergyKWh);
  }

  const bool isDark = pvWatts <= DARK_PV_WATTS_THRESHOLD;
  if (isDark) {
    if (darkStartedMs == 0) {
      darkStartedMs = now;
    }
    if (now - darkStartedMs >= NEW_DAY_DARK_MS) {
      dailyResetArmed = true;
    }
  } else if (dailyResetArmed && pvWatts >= DAY_START_PV_WATTS_THRESHOLD) {
    metrics.dailyWh = 0.0f;
    hasDailyEnergySample = false;
    dailyResetArmed = false;
    darkStartedMs = 0;
  } else if (!dailyResetArmed) {
    darkStartedMs = 0;
  }

  if (hasDailyEnergySample) {
    const uint32_t elapsedMs = now - lastDailyEnergySampleMs;
    if (elapsedMs <= MAX_DAILY_INTEGRATION_GAP_MS) {
      const float averageWatts = (lastDailyEnergyPvWatts + pvWatts) / 2.0f;
      const float addedWh = averageWatts * (elapsedMs / 3600000.0f);
      metrics.dailyWh += addedWh;
      if (isnan(metrics.totalKWh)) {
        metrics.totalKWh = static_cast<float>(storedTotalEnergyKWh);
      }
      metrics.totalKWh += addedWh / 1000.0f;
      const uint32_t totalWholeKWh = static_cast<uint32_t>(metrics.totalKWh);
      if (totalWholeKWh > storedTotalEnergyKWh && totalWholeKWh <= MAX_REASONABLE_TOTAL_ENERGY_KWH) {
        storedTotalEnergyKWh = totalWholeKWh;
        prefs.putUInt("totalKWh", storedTotalEnergyKWh);
      }
    }
  }

  lastDailyEnergySampleMs = now;
  lastDailyEnergyPvWatts = pvWatts;
  hasDailyEnergySample = true;
}

void updateMetricsFromResponse(uint8_t function, const uint16_t *regs, size_t count) {
  if (function == 0xDA && count >= 20) {
    const uint32_t now = millis();
    metrics.batteryVolts = regs[0] / 10.0f;
    metrics.pvVolts = regs[1] / 10.0f;
    metrics.estimatedBatteryAmps = regs[2] / 10.0f;
    metrics.pvWattsRaw = regs[3];
    metrics.pvAmps = metrics.pvVolts > 0.0f
                       ? (metrics.batteryVolts * metrics.estimatedBatteryAmps * PV_CURRENT_LOSS_FACTOR) / metrics.pvVolts
                       : 0.0f;
    metrics.pvWatts = metrics.pvVolts * metrics.pvAmps;
    if (!isfinite(metrics.pvWatts) || metrics.pvWatts < 0.0f) {
      metrics.pvAmps = 0.0f;
      metrics.pvWatts = 0.0f;
    }
    metrics.chargeStateRaw = regs[4];
    metrics.deviceTempC = regs[6];
    metrics.batterySocPercent = static_cast<float>(regs[9]);
    metrics.chargeActive = regs[11] != 0;
    metrics.loadFlags = regs[12];
    updateDailyEnergyEstimate(metrics.pvWatts, now);
    metrics.lastUpdateMs = now;
    metrics.decodedFrames++;
  }
}

void parseFrame(FrameBuffer &buffer, const uint8_t *data, size_t length) {
  buffer.frames++;
  buffer.lastFrameHex = hexFrame(data, length);
  buffer.lastFrameCrcOk = frameCrcOk(data, length);
  if (!buffer.lastFrameCrcOk) {
    buffer.crcErrors++;
  }

  String line;
  line.reserve(180);
  line += millis();
  line += " ";
  line += buffer.name;
  line += " len=";
  line += length;
  line += buffer.lastFrameCrcOk ? " crc=ok " : " crc=BAD ";
  line += buffer.lastFrameHex;

  if (buffer.lastFrameCrcOk && length >= 5) {
    const uint8_t function = data[1];
    const uint8_t byteCount = data[2];
    if ((function == 0x03 || function == 0xDA) && byteCount == length - 5 && (byteCount % 2) == 0) {
      uint16_t regs[24];
      const size_t count = min<size_t>(byteCount / 2, 24);
      for (size_t i = 0; i < count; ++i) {
        regs[i] = (static_cast<uint16_t>(data[3 + i * 2]) << 8) | data[4 + i * 2];
      }
      updateMetricsFromResponse(function, regs, count);
      if (function == 0xDA && count >= 16) {
        line += " pvV=";
        line += String(regs[1] / 10.0f, 1);
        line += " pvA=";
        line += String(metrics.pvAmps, 1);
        line += " pvW=";
        line += String(metrics.pvWatts, 0);
        line += " battA=";
        line += String(metrics.estimatedBatteryAmps, 1);
        line += " battV=";
        line += String(regs[0] / 10.0f, 1);
        line += " batt%=";
        line += regs[9];
        line += " tempC=";
        line += regs[6];
        line += " dailyWh=";
        line += String(metrics.dailyWh, 0);
      } else if (function == 0x03 && count >= 1) {
        line += " totalKWh=";
        line += String(metrics.totalKWh, 1);
      }
    } else if (function == 0x06 && length == 8) {
      const uint16_t reg = (static_cast<uint16_t>(data[2]) << 8) | data[3];
      const uint16_t value = (static_cast<uint16_t>(data[4]) << 8) | data[5];
      line += " write reg=0x";
      if (reg < 0x1000) {
        line += '0';
      }
      if (reg < 0x0100) {
        line += '0';
      }
      if (reg < 0x0010) {
        line += '0';
      }
      line += String(reg, HEX);
      line += " value=";
      line += value;
    }
  }

  Serial.println(line);
  telnetPrintln(line);
}

void pollSerial(FrameBuffer &buffer) {
  const uint32_t now = millis();
  while (buffer.serial->available()) {
    const int value = buffer.serial->read();
    if (value < 0) {
      break;
    }
    if (buffer.length == 0) {
      buffer.firstByteMs = now;
    }
    if (buffer.length < MAX_FRAME) {
      buffer.data[buffer.length++] = static_cast<uint8_t>(value);
      buffer.lastByteMs = now;
    } else {
      buffer.length = 0;
      buffer.crcErrors++;
    }
  }

  if (buffer.length > 0 && now - buffer.lastByteMs >= FRAME_GAP_MS) {
    parseFrame(buffer, buffer.data, buffer.length);
    buffer.length = 0;
  }
}

String statusJson() {
  String out;
  out.reserve(1300);
  out += "{";
  out += "\"uptimeMs\":";
  out += millis();
  out += ",\"wifi\":{";
  out += "\"phase\":\"";
  out += wifiPhaseName();
  out += "\",\"apRunning\":";
  out += apRunning ? "true" : "false";
  out += ",\"apIp\":\"";
  out += apRunning ? WiFi.softAPIP().toString() : "";
  out += "\",\"staConnected\":";
  out += WiFi.status() == WL_CONNECTED ? "true" : "false";
  out += ",\"staIp\":\"";
  out += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  out += "\",\"ssid\":\"";
  out += jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.SSID() : savedSsid);
  out += "\",\"rssi\":";
  out += WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : "null";
  out += ",\"phaseRemainingSec\":";
  const uint32_t now = millis();
  uint32_t remaining = 0;
  if (wifiPhase == WifiPhase::StaRetry) {
    remaining = now - wifiPhaseStartedMs >= STA_RETRY_WINDOW_MS ? 0 : (STA_RETRY_WINDOW_MS - (now - wifiPhaseStartedMs)) / 1000;
  } else if (wifiPhase == WifiPhase::ApReconfig) {
    remaining = now - wifiPhaseStartedMs >= AP_RECONFIG_WINDOW_MS ? 0 : (AP_RECONFIG_WINDOW_MS - (now - wifiPhaseStartedMs)) / 1000;
  }
  out += remaining;
  out += "},\"metrics\":{";
  appendJsonFloat(out, "batteryVolts", metrics.batteryVolts);
  out += ",";
  appendJsonFloat(out, "pvVolts", metrics.pvVolts);
  out += ",";
  appendJsonFloat(out, "pvAmps", metrics.pvAmps);
  out += ",";
  appendJsonFloat(out, "pvWatts", metrics.pvWatts, 0);
  out += ",";
  appendJsonFloat(out, "estimatedBatteryAmps", metrics.estimatedBatteryAmps);
  out += ",\"pvCurrentLossFactor\":";
  out += String(PV_CURRENT_LOSS_FACTOR, 4);
  out += ",";
  appendJsonFloat(out, "batterySocPercent", metrics.batterySocPercent, 0);
  out += ",";
  appendJsonFloat(out, "dailyWh", metrics.dailyWh, 0);
  out += ",";
  appendJsonFloat(out, "totalKWh", metrics.totalKWh, 1);
  out += ",\"pvWattsRaw\":";
  out += metrics.pvWattsRaw;
  out += ",\"chargeStateRaw\":";
  out += metrics.chargeStateRaw;
  out += ",\"chargeActive\":";
  out += metrics.chargeActive ? "true" : "false";
  out += ",";
  appendJsonInt(out, "deviceTempC", metrics.deviceTempC);
  out += ",\"ageSec\":";
  out += metrics.lastUpdateMs ? String((millis() - metrics.lastUpdateMs) / 1000) : "null";
  out += ",\"decodedFrames\":";
  out += metrics.decodedFrames;
  out += "},\"serial\":{";
  out += "\"controllerFrames\":";
  out += controllerRx.frames;
  out += ",\"controllerCrcErrors\":";
  out += controllerRx.crcErrors;
  out += ",\"displayFrames\":";
  out += displayRx.frames;
  out += ",\"displayCrcErrors\":";
  out += displayRx.crcErrors;
  out += ",\"lastControllerFrame\":\"";
  out += jsonEscape(controllerRx.lastFrameHex);
  out += "\",\"lastDisplayFrame\":\"";
  out += jsonEscape(displayRx.lastFrameHex);
  out += "\"}}";
  return out;
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MPPT Monitor</title>
<style>
:root{font-family:Inter,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:#17211d;background:#f6f7f4}
body{margin:0}
header{display:flex;align-items:center;justify-content:space-between;gap:16px;padding:16px 20px;background:#ffffff;border-bottom:1px solid #d9ded6;position:sticky;top:0}
h1{font-size:20px;line-height:1.1;margin:0;font-weight:700}
main{max-width:1120px;margin:0 auto;padding:20px;display:grid;gap:18px}
.status{font-size:13px;color:#4a564f}
.dashboard{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));grid-template-areas:"pvV pvA pvW batV temp" "batA daily total . .";gap:12px;align-items:stretch}
.pv-volts{grid-area:pvV}.pv-amps{grid-area:pvA}.pv-watts{grid-area:pvW}.battery-volts{grid-area:batV}.battery-amps{grid-area:batA}.daily-energy{grid-area:daily}.total-energy{grid-area:total}.mosfet-temp{grid-area:temp}
.card,.panel{background:#fff;border:1px solid #d9ded6;border-radius:8px;padding:14px;min-width:0}
.label{font-size:12px;text-transform:uppercase;letter-spacing:.04em;color:#5d685f}
.value{font-size:30px;font-weight:750;margin-top:5px;font-variant-numeric:tabular-nums}
.unit{font-size:16px;color:#68736b;margin-left:3px}
.panel h2{font-size:16px;margin:0 0 12px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
input,button{font:inherit;border-radius:7px;border:1px solid #bbc5bd;background:#fff;padding:9px 10px}
button{background:#1e6b55;color:white;border-color:#1e6b55;cursor:pointer}
button.secondary{background:#f8faf7;color:#26302a;border-color:#bbc5bd}
button:disabled{opacity:.55;cursor:default}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:8px;border-bottom:1px solid #e3e8e1;text-align:left}
@media(max-width:760px){.dashboard{grid-template-columns:minmax(0,1fr) minmax(0,1fr);grid-template-areas:"pvV batV" "pvW batA" "pvA daily" "temp total";gap:12px}}
@media(max-width:460px){main{padding:12px}.card,.panel{padding:12px}.value{font-size:26px}.unit{font-size:14px}header{align-items:flex-start;flex-direction:column}}
</style>
</head>
<body>
<header>
  <h1>MPPT Monitor</h1>
  <div class="status" id="net">starting</div>
</header>
<main>
  <section class="dashboard">
    <div class="card pv-volts"><div class="label">PV Volts</div><div class="value"><span id="pvV">--</span><span class="unit">V</span></div></div>
    <div class="card pv-amps"><div class="label">PV Amps</div><div class="value"><span id="pvA">--</span><span class="unit">A</span></div></div>
    <div class="card pv-watts"><div class="label">PV Watts</div><div class="value"><span id="pvW">--</span><span class="unit">W</span></div></div>
    <div class="card battery-volts"><div class="label">Battery Volts</div><div class="value"><span id="batV">--</span><span class="unit">V</span></div></div>
    <div class="card mosfet-temp"><div class="label">MOSFET Temp</div><div class="value"><span id="temp">--</span><span class="unit">C</span></div></div>
    <div class="card battery-amps"><div class="label">Battery Amps</div><div class="value"><span id="batA">--</span><span class="unit">A</span></div></div>
    <div class="card daily-energy"><div class="label">Daily Energy</div><div class="value"><span id="dailyWh">--</span><span class="unit">Wh</span></div></div>
    <div class="card total-energy"><div class="label">Total Energy</div><div class="value"><span id="totalKWh">--</span><span class="unit">kWh</span></div></div>
  </section>

  <section class="panel">
    <h2>WiFi</h2>
    <div class="row">
      <button id="scan">Scan</button>
      <input id="ssid" placeholder="SSID" autocomplete="off">
      <input id="pass" placeholder="Password" type="password">
      <button id="connect">Connect</button>
      <button class="secondary" id="forget">Forget</button>
    </div>
    <table id="networks" style="margin-top:10px"></table>
  </section>

</main>
<script>
const $ = id => document.getElementById(id);
const fmt = (v, d=1) => v === null || v === undefined ? "--" : Number(v).toFixed(d);
async function status(){
  const r = await fetch("/api/status", {cache:"no-store"});
  const s = await r.json();
  $("pvV").textContent = fmt(s.metrics.pvVolts);
  $("pvA").textContent = fmt(s.metrics.pvAmps);
  $("pvW").textContent = fmt(s.metrics.pvWatts,0);
  $("batA").textContent = fmt(s.metrics.estimatedBatteryAmps);
  $("batV").textContent = fmt(s.metrics.batteryVolts);
  $("temp").textContent = s.metrics.deviceTempC ?? "--";
  $("dailyWh").textContent = fmt(s.metrics.dailyWh,0);
  $("totalKWh").textContent = fmt(s.metrics.totalKWh,1);
  const ip = s.wifi.staConnected ? s.wifi.staIp : s.wifi.apIp;
  $("net").textContent = `${s.wifi.phase} ${ip || ""} ${s.metrics.ageSec === null ? "" : "data " + s.metrics.ageSec + "s ago"}`;
}
async function scan(){
  $("scan").disabled = true;
  $("networks").innerHTML = "<tr><td>Scanning...</td></tr>";
  try {
    const r = await fetch("/api/wifi/scan");
    const nets = await r.json();
    $("networks").innerHTML = "<tr><th>SSID</th><th>RSSI</th><th></th></tr>" + nets.map(n =>
      `<tr><td>${n.ssid}</td><td>${n.rssi}</td><td><button class="secondary" data-ssid="${n.ssid}">Use</button></td></tr>`
    ).join("");
    document.querySelectorAll("[data-ssid]").forEach(b => b.onclick = () => $("ssid").value = b.dataset.ssid);
  } finally {
    $("scan").disabled = false;
  }
}
async function connectWifi(){
  const body = new URLSearchParams({ssid:$("ssid").value, password:$("pass").value});
  await fetch("/api/wifi/connect", {method:"POST", body});
  $("net").textContent = "saved, switching to station mode";
}
async function forgetWifi(){
  await fetch("/api/wifi/forget", {method:"POST"});
  $("net").textContent = "credentials cleared";
}
$("scan").onclick = scan;
$("connect").onclick = connectWifi;
$("forget").onclick = forgetWifi;
status();
setInterval(status, 1000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", statusJson());
}

void handleWifiScan() {
  const int networks = WiFi.scanNetworks(false, true);
  String out = "[";
  for (int i = 0; i < networks; ++i) {
    if (i) {
      out += ",";
    }
    out += "{\"ssid\":\"";
    out += jsonEscape(WiFi.SSID(i));
    out += "\",\"rssi\":";
    out += WiFi.RSSI(i);
    out += ",\"secure\":";
    out += WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true";
    out += "}";
  }
  out += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", out);
}

void handleWifiConnect() {
  const String ssid = server.arg("ssid");
  const String password = server.arg("password");
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "missing ssid");
    return;
  }
  saveCredentials(ssid, password);
  server.send(200, "text/plain", "saved");
  delay(100);
  enterStaRetry();
}

void handleWifiForget() {
  clearCredentials();
  WiFi.disconnect(true, true);
  server.send(200, "text/plain", "forgot");
  delay(100);
  enterSetupAp();
}

void handleNotFound() {
  server.send(404, "text/plain", "not found");
}

void setupWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/connect", HTTP_POST, handleWifiConnect);
  server.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
  server.onNotFound(handleNotFound);
  server.begin();
}

void pollTelnet() {
  if (!telnetClient || !telnetClient.connected()) {
    WiFiClient incoming = telnetServer.accept();
    if (incoming) {
      if (telnetClient) {
        telnetClient.stop();
      }
      telnetClient = incoming;
      telnetClient.setNoDelay(true);
      telnetClient.println("MPPT telnet. Commands: help, status, frames");
    }
    return;
  }

  while (telnetClient.available()) {
    const char c = static_cast<char>(telnetClient.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      telnetLine.trim();
      if (telnetLine == "help") {
        telnetClient.println("help, status, frames");
      } else if (telnetLine == "status") {
        telnetClient.println(statusJson());
      } else if (telnetLine == "frames") {
        telnetClient.print("controller: ");
        telnetClient.println(controllerRx.lastFrameHex);
        telnetClient.print("display: ");
        telnetClient.println(displayRx.lastFrameHex);
      } else if (telnetLine.length()) {
        telnetClient.println("unknown command");
      }
      telnetLine = "";
    } else if (telnetLine.length() < 80) {
      telnetLine += c;
    }
  }
}

void setupSerialSniffers() {
  controllerSerial.begin(SERIAL_BAUD, SERIAL_8N1, RX_CONTROLLER_PIN, -1);
  displaySerial.begin(SERIAL_BAUD, SERIAL_8N1, RX_DISPLAY_PIN, -1);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("MPPT monitor booting");

  loadCredentials();
  loadEnergyState();
  setupSerialSniffers();

  if (hasCredentials) {
    enterStaRetry();
  } else {
    enterSetupAp();
  }

  setupWeb();
  telnetServer.begin();
  telnetServer.setNoDelay(true);
}

void loop() {
  pollSerial(controllerRx);
  pollSerial(displayRx);
  server.handleClient();
  pollTelnet();
  pollWifiState();
}
