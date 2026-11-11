"""UART peer for bridge bring-up. No actuators; every control request is rejected.

Requires pyserial and cbor2 only in --port mode. --example works with the standard library.
"""
import argparse
import json
import random
import struct
import time
import zlib

VERSION = 2  # BLE
UART_VERSION = 3
MAX_RAW = 1038
HELLO, SCHEMA_REQUEST, SCHEMA, SNAPSHOT_REQUEST = 1, 2, 3, 4
TELEMETRY, ERRORS, COMMAND, STOP, COMMAND_RESULT, LINK_STATE = 0x10, 0x11, 0x20, 0x21, 0x22, 0x40


def json_payload(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")


def raw_packet(kind, sequence, payload, version=VERSION):
    if len(payload) > 1024:
        raise ValueError("Payload exceeds 1024 bytes")
    head = struct.pack("<BBHHI", version, kind, 0, len(payload), sequence)
    data = head + payload
    return data + struct.pack("<I", zlib.crc32(data))


def parse_packet(raw, expected_version=VERSION):
    if not 14 <= len(raw) <= MAX_RAW:
        raise ValueError("Invalid raw length")
    version, kind, flags, length, sequence = struct.unpack_from("<BBHHI", raw)
    if version != expected_version or flags or len(raw) != 14 + length:
        raise ValueError("Invalid header")
    if zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, len(raw) - 4)[0]:
        raise ValueError("CRC mismatch")
    return kind, sequence, raw[10:-4]


def cobs_encode(data):
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 255:
                out[code_index] = code
                code_index, code = len(out), 1
                out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    out, pos = bytearray(), 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if not code or pos + code - 1 > len(data):
            raise ValueError("Invalid COBS block")
        block = data[pos:pos + code - 1]
        if 0 in block:
            raise ValueError("Zero in COBS block")
        out.extend(block)
        pos += code - 1
        if code != 255 and pos < len(data):
            out.append(0)
    return bytes(out)


def fragments(raw, message_id=1, mtu=23):
    step = mtu - 3 - 6
    if not 23 <= mtu <= 247:
        raise ValueError("MTU must be 23..247")
    return [struct.pack("<HHH", message_id, len(raw), offset) + raw[offset:offset + step]
            for offset in range(0, len(raw), step)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB-UART adapter connected to ESP32 GPIO17/18")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--example", action="store_true", help="Print frames without hardware")
    args = parser.parse_args()
    if args.example:
        for sequence, (label, kind, payload) in enumerate([
            ("phone_snapshot_request", SNAPSHOT_REQUEST, {}),
            ("phone_simulated_start", COMMAND, {"op": "start"}),
            ("phone_simulated_stop", STOP, {}),
            ("stm32_telemetry", TELEMETRY, {"boot_id": 42, "sample_ms": 1000, "values": {"drive_battery_voltage": 24700, "drive_battery_temperature": 35}}),
        ], start=1):
            raw = raw_packet(kind, sequence, json_payload(payload))
            print(json.dumps({"name": label, "payload": payload, "raw_hex": raw.hex(),
                              "ble_mtu23_hex": [f.hex() for f in fragments(raw, message_id=sequence)]}, indent=2))
        return
    if not args.port:
        parser.error("Specify --port COMx or --example")
    try:
        import serial
        import cbor2
    except ImportError:
        raise SystemExit("Install UART simulator dependencies: python -m pip install pyserial cbor2")

    boot_id = random.getrandbits(32)
    sequence = 0
    started = time.monotonic()
    schema = {"revision": 1, "part_index": 0, "parts_total": 1, "fields": [
        {"id": 1001, "key": "drive_battery_voltage", "name": "Battery voltage",
         "type": "uint32", "unit": "mV", "access": "read", "period_ms": 1000},
        {"id": 1002, "key": "drive_battery_temperature", "name": "Battery temperature",
         "type": "int32", "unit": "degC", "access": "read", "period_ms": 1000},
    ]}
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1) as port:
        def send(kind, value, request_sequence=None):
            nonlocal sequence
            sequence = (sequence + 1) & 0xffffffff
            seq = sequence if request_sequence is None else request_sequence
            port.write(cobs_encode(raw_packet(kind, seq, cbor2.dumps(value), UART_VERSION)) + b"\0")

        def snapshot():
            elapsed = int((time.monotonic() - started) * 1000) & 0xffffffff
            send(TELEMETRY, {"boot_id": boot_id, "sample_ms": elapsed,
                             "values": {"drive_battery_voltage": 24700, "drive_battery_temperature": 35}})
            send(ERRORS, {"boot_id": boot_id, "revision": 0, "active": []})

        print(f"STM32 simulator on {args.port}; commands are rejected; Ctrl+C to exit.")
        port.write(b"\0")
        buffer, discard = bytearray(), False
        next_tick = time.monotonic()
        while True:
            for byte in port.read(2048):
                if byte:
                    if not discard:
                        buffer.append(byte)
                        if len(buffer) > 1043:
                            buffer.clear()
                            discard = True
                    continue
                if discard or not buffer:
                    buffer.clear()
                    discard = False
                    continue
                try:
                    kind, request_seq, payload = parse_packet(cobs_decode(buffer), UART_VERSION)
                    value = cbor2.loads(payload)
                    if not isinstance(value, dict):
                        raise ValueError("CBOR payload must be a map")
                    print(f"RX type=0x{kind:02x} seq={request_seq} payload={value}")
                    if kind == HELLO:
                        send(HELLO, {"device_id": "stm32-simulator", "boot_id": boot_id, "schema_revision": 1})
                    elif kind == SCHEMA_REQUEST:
                        send(SCHEMA, schema)
                    elif kind == SNAPSHOT_REQUEST:
                        snapshot()
                    elif kind in (COMMAND, STOP):
                        send(COMMAND_RESULT, {"status": "rejected", "reason": "simulator_no_actuators"}, request_seq)
                    elif kind == LINK_STATE:
                        pass  # Link state never substitutes for a manual-control lease.
                except (ValueError, cbor2.CBORDecodeError) as error:
                    print(f"Rejected UART frame: {error}")
                buffer.clear()
            if time.monotonic() >= next_tick:
                snapshot()
                next_tick = time.monotonic() + 1


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
