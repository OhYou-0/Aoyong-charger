# MPPT COM6 serial notes

COM6 was passively sampled as an RX-only serial tap between the MPPT charger display and controller.

## Result

- Baud/settings: `9600 8N1`, no flow control.
- Protocol family: Modbus/RTU-like framing because the final two bytes are a valid Modbus CRC-16, low byte first.
- Traffic seen: two short frames alternating roughly once per second.
- Important limitation: the tap only shows these short frames. I do not see a data-heavy controller response on COM6, so this may be only one direction of a two-wire TTL UART link. To decode live charger values, also tap the other TX line or use a proper differential RS485 tap if the link is RS485.

## Repeating frames

```text
FF 03 00 00 00 0A 14 12 93
FF DA 00 00 00 14 28 01 EB 06
```

Both pass Modbus CRC-16:

```text
crc16(FF 03 00 00 00 0A 14)       = 0x9312, sent as 12 93
crc16(FF DA 00 00 00 14 28 01)    = 0x06EB, sent as EB 06
```

The first frame is read-like:

```text
addr=0xFF func=0x03 start=0x0000 qty=10 expected_bytes=20
```

That extra `0x14` byte equals 20 decimal, which is the byte count for 10 16-bit registers. Stock Modbus read requests do not include that byte, so this is probably a vendor extension or display-side command format rather than plain Modbus.

## Other direction

When COM6 is connected to the other wire, the traffic is still `9600 8N1` with valid Modbus CRC-16, but the frames are now the data-bearing responses:

```text
FF 03 14 00 01 00 00 00 64 01 B8 02 40 02 20 00 03 01 CC 00 01 00 00 7B 68
FF DA 28 01 DB 00 00 00 00 00 00 00 00 00 00 00 19 00 14 00 FF 00 21 00 04 00 00 00 00 00 3C 00 AB 00 28 02 38 02 20 01 CC 01 B8 44 76
```

The `0x03` response contains 10 big-endian 16-bit registers:

```text
1, 0, 100, 440, 576, 544, 3, 460, 1, 0
```

The `0xDA` response contains 20 big-endian 16-bit registers:

```text
475, 0, 0, 0, 0, 0, 25, 20, 255, 33, 4, 0, 0, 60, 171, 40, 568, 544, 460, 440
```

Several values look like tenths-scaled charger values or setpoints. For example `475` may be `47.5`, and `568`, `544`, `460`, `440` look like voltage thresholds on a 48 V battery profile.

## Toggle capture

Toggling the display load switch produced valid Modbus function `0x06` frames:

```text
FF 06 00 64 00 01 1C 0B  # write register 0x0064 = 1
FF 06 00 64 00 00 DD CB  # write register 0x0064 = 0
```

The next response frames changed in two places:

```text
0x03 response register 9:   0 -> 1 -> 0
0xDA response register 12:  0 -> 129 -> 0
```

Current working interpretation:

| Frame | Register index | Meaning |
| --- | ---: | --- |
| `0xDA` | 0 | Battery voltage, x10. Observed `475` = `47.5 V`. |
| `0xDA` | 1 | Solar/PV input voltage, x10. Observed `119` = `11.9 V`. |
| `0xDA` | 2 | Battery/output current, x10. Confirmed by meter check; used directly as Battery Amps in firmware. |
| `0xDA` | 3 | Watts raw / unused. Observed `0` even while charging, so firmware calculates PV watts as `PV V * PV A`. |
| `0xDA` | 4 | Charge state raw. Observed `0` idle, `16` while charging. |
| `0xDA` | 5 | Reserved / unknown. Observed `0`. |
| `0xDA` | 6 | Controller/device temperature. Observed `25` = `25 C`. |
| `0xDA` | 8 | Battery temperature raw / unavailable marker. Observed `255` while display says `n/a`. |
| `0xDA` | 9 | Likely battery state of charge percent. Observed `33` on the 48 V bench capture and `86` to `87` installed. |
| `0xDA` | 10 | System voltage code. Observed `4` on the 48 V bench capture and `11` installed. |
| `0xDA` | 11 | Charge active flag. Observed `0` idle, `1` while charging. |
| `0xDA` | 12 | Load output state/flags. Observed `0` off, `129` on. |
| `0xDA` | 16 | Battery voltage setting, x10. Observed `568` = `56.8 V`. |
| `0xDA` | 17 | Battery voltage setting, x10. Observed `544` = `54.4 V`. |
| `0xDA` | 18 | Reconnect/return voltage setting, x10. Observed `460` = `46.0 V`. |
| `0xDA` | 19 | Low-voltage disconnect or related setting, x10. Observed `440` = `44.0 V`. |
| `0x03` | 9 | Load switch command state. Observed `0` off, `1` on. |

The `0x03` response also mirrors several battery voltage settings: `440`, `576`, `544`, and `460`.

## Solar input capture

Applying about `11.9 V` to the solar input changed only `0xDA` register 1:

```text
0xDA register 1 observed values: 0, 29, 51, 53, 59, 71, 119
```

Interpreted as x10 volts, those are `0.0`, `2.9`, `5.1`, `5.3`, `5.9`, `7.1`, and `11.9 V`. No current or wattage fields moved, which is expected because an `11.9 V` PV input cannot charge a roughly `47.5 V` battery pack.

## Installed charging behavior

A live installed sample plus meter check confirmed the current firmware interpretation:

```text
0xDA[0] battery voltage, x10
0xDA[1] PV voltage, x10
0xDA[2] battery/output current, x10
0xDA[6] MOSFET/controller temperature, degrees C
0xDA[9] likely battery SOC percent
```

`0xDA[3]` stayed `0` while the charger was active, so it is not a usable wattage field. The ESP32 firmware displays battery amps directly from `0xDA[2] / 10`.

PV amps are estimated from battery-side power and PV voltage:

```text
pv amps = battery volts * battery amps / pv volts * 1.0526
pv watts = pv volts * pv amps
```

There is no other obvious battery-current field in the observed live samples. The only dynamic non-voltage fields were:

```text
0xDA[2]     battery/output current, x10
0xDA[6]     controller temperature
0xDA[9]     likely battery SOC percent
```

Treat `0xDA[2]` as battery/output current. PV input current is estimated from battery power and PV voltage.

The evidence is summarized in `mppt_field_observations.csv` so future snippets can be compared against the current baseline. Two still-unknown values to hunt with longer spans are daily Wh and total kWh. They may only move slowly, so compare captures separated by hours or days.

## Energy register candidates

Short live samples were compared against display daily and lifetime energy readings. The currently observed `0xDA` and `0x03` response payloads did not contain an obvious matching value.

| Frame | Register index | Candidate |
| --- | ---: | --- |
| `0xDA` | 15 | Rejected daily energy candidate; it did not track the display daily value. |
| `0x03` | 0 | Rejected total/lifetime energy candidate; it did not track the display total value. |

The firmware now estimates daily Wh and total kWh locally by integrating calculated PV watts over elapsed time.

Daily energy resets after PV has been at or below `5 W` for at least one hour, then later rises to at least `200 W`.

Total energy is stored in ESP32 Preferences as whole kWh only. If no stored value exists, firmware creates a `0 kWh` checkpoint. If a stored value already exists, firmware respects it and starts the live display at that whole-kWh value. The web UI updates from RAM at `0.1 kWh` resolution. Flash is only written when the floored total kWh increases, so a power loss at e.g. `12.9 kWh` recovers from the last written whole value, `12 kWh`.

## Usage

Scan:

```powershell
python .\mppt_sniffer.py scan --port COM6 --seconds 2
```

Live decode:

```powershell
python .\mppt_sniffer.py sniff --port COM6 --seconds 30
```

Log CSV:

```powershell
python .\mppt_sniffer.py sniff --port COM6 --csv mppt_frames.csv
```

## ESP32 monitor firmware

The firmware project in `src/main.cpp` targets a Seeed XIAO ESP32-S3 with PlatformIO environment `seeed_xiao_esp32s3`.

Wiring:

| XIAO pin | GPIO | Connect to |
| --- | ---: | --- |
| `D7 / RX` | `GPIO44` | Controller-side `TX`, the value/response direction. |
| `D1` | `GPIO2` | Display-side `TX`, the request/write direction. |
| `GND` |  | Charger/display ground. |

Do not connect an ESP32 TX pin to the charger yet. The firmware is a passive sniffer.

The XIAO ESP32-S3 inputs are 3.3 V logic. If the charger serial lines are 5 V TTL, use a level shifter or a simple divider on each RX input before connecting to the ESP32.

Network behavior:

- First boot with no saved WiFi credentials starts AP `mppt` with password `password`.
- AP IP is `192.168.4.1` with DHCP.
- The web UI can scan and save existing WiFi credentials.
- After saving credentials, the AP stops and the ESP32 connects as a station.
- If station WiFi is lost, firmware cycles through 2 minutes of station reconnect attempts, then 5 minutes of AP reconfiguration availability.
- Telnet on port `23` streams decoded frames and accepts `help`, `status`, and `frames`.
