#!/usr/bin/env python3
"""Passive sniffer for the MPPT display/controller serial link.

The observed link on COM6 is 9600 baud, 8N1. Frames use the Modbus/RTU
CRC-16, but the payloads are not plain stock Modbus request/response frames.
This script never writes to the serial port.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import serial


COMMON_BAUDS = (300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200)
PV_CURRENT_LOSS_FACTOR = 1.0526

DA_REGISTER_LABELS = {
    0: "battery_voltage_x10",
    1: "solar_input_voltage_x10",
    2: "battery_current_x10",
    3: "solar_input_watts_raw_or_unused",
    4: "charge_state_raw",
    5: "reserved_or_zero",
    6: "device_temp_c",
    8: "battery_temp_raw_or_na",
    9: "battery_soc_percent",
    10: "system_voltage_code",
    11: "charge_active_flag",
    12: "load_output_state_flags",
    16: "boost_voltage_x10",
    17: "equalize_voltage_x10",
    18: "reconnect_voltage_x10",
    19: "low_voltage_disconnect_x10",
}

FUNC03_REGISTER_LABELS = {
    3: "low_voltage_disconnect_x10",
    4: "boost_voltage_x10",
    5: "equalize_voltage_x10",
    7: "reconnect_voltage_x10",
    9: "load_switch_command_state",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc_ok(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    expected = frame[-2] | (frame[-1] << 8)
    return crc16_modbus(frame[:-2]) == expected


@dataclass(frozen=True)
class Frame:
    timestamp: float
    raw: bytes

    @property
    def crc_ok(self) -> bool:
        return crc_ok(self.raw)

    @property
    def address(self) -> int | None:
        return self.raw[0] if self.raw else None

    @property
    def function(self) -> int | None:
        return self.raw[1] if len(self.raw) > 1 else None

    @property
    def payload(self) -> bytes:
        return self.raw[2:-2] if len(self.raw) >= 4 else b""


def open_port(port: str, baud: int, parity: str = "N") -> serial.Serial:
    ser = serial.Serial(
        port,
        baudrate=baud,
        bytesize=8,
        parity=parity,
        stopbits=1,
        timeout=0.02,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    # Keep control lines quiet. The adapter is RX-only, but this avoids surprises
    # on adapters that expose DTR/RTS pins.
    ser.dtr = False
    ser.rts = False
    return ser


def capture_frames(port: str, baud: int, seconds: float, parity: str = "N", gap_ms: float = 50.0) -> list[Frame]:
    ser = open_port(port, baud, parity)
    events: list[tuple[float, int]] = []
    start = time.monotonic()
    try:
        while time.monotonic() - start < seconds:
            byte = ser.read(1)
            if byte:
                events.append((time.monotonic() - start, byte[0]))
    finally:
        ser.close()

    frames: list[Frame] = []
    current: list[tuple[float, int]] = []
    previous_time: float | None = None
    gap_s = gap_ms / 1000.0
    for timestamp, byte in events:
        if previous_time is not None and timestamp - previous_time > gap_s and current:
            frames.append(Frame(current[0][0], bytes(b for _, b in current)))
            current = []
        current.append((timestamp, byte))
        previous_time = timestamp
    if current:
        frames.append(Frame(current[0][0], bytes(b for _, b in current)))
    return frames


def iter_frames(
    port: str,
    baud: int,
    seconds: float | None = None,
    parity: str = "N",
    gap_ms: float = 50.0,
):
    ser = open_port(port, baud, parity)
    start = time.monotonic()
    gap_s = gap_ms / 1000.0
    current = bytearray()
    first_timestamp: float | None = None
    previous_timestamp: float | None = None
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if seconds is not None and elapsed >= seconds:
                break

            byte = ser.read(1)
            now = time.monotonic()
            elapsed = now - start
            if byte:
                if previous_timestamp is not None and elapsed - previous_timestamp > gap_s and current:
                    yield Frame(first_timestamp or 0.0, bytes(current))
                    current = bytearray()
                    first_timestamp = elapsed
                elif not current:
                    first_timestamp = elapsed
                current.append(byte[0])
                previous_timestamp = elapsed
            elif current and previous_timestamp is not None and elapsed - previous_timestamp > gap_s:
                yield Frame(first_timestamp or 0.0, bytes(current))
                current = bytearray()
                first_timestamp = None
                previous_timestamp = None
    finally:
        ser.close()

    if current:
        yield Frame(first_timestamp or 0.0, bytes(current))


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def parse_frame(frame: Frame) -> str:
    raw = frame.raw
    if len(raw) < 4:
        return "too short"

    address = raw[0]
    function = raw[1]
    payload = raw[2:-2]
    crc = raw[-2] | (raw[-1] << 8)
    parts = [
        f"addr=0x{address:02X}",
        f"func=0x{function:02X}",
        f"payload={hex_bytes(payload) or '-'}",
        f"crc=0x{crc:04X}",
        "crc_ok" if frame.crc_ok else "crc_BAD",
    ]

    if payload and payload[0] == len(payload) - 1 and (len(payload) - 1) % 2 == 0:
        data = payload[1:]
        words = [(data[index] << 8) | data[index + 1] for index in range(0, len(data), 2)]
        scaled_x10 = [f"{value / 10:.1f}" for value in words]
        labels = DA_REGISTER_LABELS if function == 0xDA else FUNC03_REGISTER_LABELS if function == 0x03 else {}
        named = []
        for index, value in enumerate(words):
            label = labels.get(index)
            if label:
                if label.endswith("_x10"):
                    named.append(f"{label}={value / 10:.1f}")
                else:
                    named.append(f"{label}={value}")
        parts.append(f"response byte_count={payload[0]}")
        parts.append("regs_be=" + ",".join(f"0x{value:04X}" for value in words))
        parts.append("regs_dec=" + ",".join(str(value) for value in words))
        parts.append("regs_x10=" + ",".join(scaled_x10))
        if function == 0xDA and len(words) > 2:
            battery_volts = words[0] / 10
            pv_volts = words[1] / 10
            battery_amps = words[2] / 10
            battery_watts = battery_volts * battery_amps
            parts.append(f"derived_battery_watts={battery_watts:.0f}")
            if pv_volts > 0:
                estimated_pv_amps = (battery_watts * PV_CURRENT_LOSS_FACTOR) / pv_volts
                parts.append(
                    f"estimated_pv_amps_loss_{PV_CURRENT_LOSS_FACTOR:.4f}="
                    f"{estimated_pv_amps:.1f}"
                )
                parts.append(f"estimated_pv_watts={pv_volts * estimated_pv_amps:.0f}")
        if named:
            parts.append("known=" + " ".join(named))
        return " ".join(parts)

    if function == 0x06 and len(payload) == 4:
        register = (payload[0] << 8) | payload[1]
        value = (payload[2] << 8) | payload[3]
        parts.append(f"write-single-register register=0x{register:04X} value={value}")
        if register == 0x0064:
            parts.append("known=load_switch_command")

    if function == 0x03 and len(payload) == 5:
        start_register = (payload[0] << 8) | payload[1]
        quantity = (payload[2] << 8) | payload[3]
        expected_bytes = payload[4]
        parts.append(
            f"read-like start=0x{start_register:04X} qty={quantity} expected_bytes={expected_bytes}"
        )
    elif function == 0x03 and len(payload) == 4:
        start_register = (payload[0] << 8) | payload[1]
        quantity = (payload[2] << 8) | payload[3]
        parts.append(f"modbus-read start=0x{start_register:04X} qty={quantity}")

    if payload:
        words_be = []
        words_le = []
        for index in range(0, len(payload) - 1, 2):
            words_be.append((payload[index] << 8) | payload[index + 1])
            words_le.append(payload[index] | (payload[index + 1] << 8))
        if words_be:
            parts.append("u16be=" + ",".join(str(value) for value in words_be))
            parts.append("u16le=" + ",".join(str(value) for value in words_le))

    return " ".join(parts)


def scan(args: argparse.Namespace) -> int:
    for baud in COMMON_BAUDS:
        for parity in ("N", "E", "O"):
            try:
                frames = capture_frames(args.port, baud, args.seconds, parity, args.gap_ms)
            except serial.SerialException as exc:
                print(f"{baud:6}{parity}: open failed: {exc}")
                continue

            total_bytes = sum(len(frame.raw) for frame in frames)
            good = sum(1 for frame in frames if frame.crc_ok)
            unique = len({frame.raw for frame in frames})
            sample = hex_bytes(frames[0].raw) if frames else "-"
            print(
                f"{baud:6}{parity}: frames={len(frames):2} bytes={total_bytes:4} "
                f"crc_ok={good:2} unique={unique:2} sample={sample}"
            )
    return 0


def sniff(args: argparse.Namespace) -> int:
    csv_file = None
    writer = None
    if args.csv:
        csv_path = Path(args.csv)
        csv_file = csv_path.open("a", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        if csv_path.stat().st_size == 0:
            writer.writerow(("wall_time", "elapsed_s", "baud", "raw_hex", "crc_ok", "parsed"))

    seen: Counter[bytes] = Counter()
    try:
        for frame in iter_frames(args.port, args.baud, args.seconds, args.parity, args.gap_ms):
            seen[frame.raw] += 1
            parsed = parse_frame(frame)
            raw_hex = hex_bytes(frame.raw)
            print(f"{frame.timestamp:8.3f}s count={seen[frame.raw]:3} len={len(frame.raw):2} {raw_hex}  {parsed}")
            if writer:
                writer.writerow(
                    (
                        time.strftime("%Y-%m-%d %H:%M:%S"),
                        f"{frame.timestamp:.3f}",
                        args.baud,
                        raw_hex,
                        frame.crc_ok,
                        parsed,
                    )
                )
                csv_file.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if csv_file:
            csv_file.close()
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Passive MPPT serial sniffer")
    subparsers = parser.add_subparsers(dest="command")

    scan_parser = subparsers.add_parser("scan", help="try common baud/parity settings")
    scan_parser.add_argument("--port", default="COM6")
    scan_parser.add_argument("--seconds", type=float, default=2.0)
    scan_parser.add_argument("--gap-ms", type=float, default=50.0)
    scan_parser.set_defaults(func=scan)

    sniff_parser = subparsers.add_parser("sniff", help="live decode frames")
    sniff_parser.add_argument("--port", default="COM6")
    sniff_parser.add_argument("--baud", type=int, default=9600)
    sniff_parser.add_argument("--parity", choices=("N", "E", "O"), default="N")
    sniff_parser.add_argument("--seconds", type=float)
    sniff_parser.add_argument("--gap-ms", type=float, default=50.0)
    sniff_parser.add_argument("--csv", help="append decoded frames to a CSV file")
    sniff_parser.set_defaults(func=sniff)

    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        parser.print_help()
        return 2
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
