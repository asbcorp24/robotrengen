"""Checks for the independently implemented Python UART peer."""
import importlib.util
from pathlib import Path
import struct
import unittest
import zlib

spec = importlib.util.spec_from_file_location(
    "peer", Path(__file__).resolve().parents[2] / "tools/stm32_simulator.py")
peer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(peer)


class PeerTests(unittest.TestCase):
    def test_golden_fixture(self):
        raw = peer.raw_packet(peer.SNAPSHOT_REQUEST, 1, peer.cbor({}))
        self.assertEqual(raw.hex(), "01040000010001000000a0ba4f23ee")
        self.assertEqual((peer.cobs_encode(raw) + b"\0").hex(),
                         "0301040102010201010106a0ba4f23ee00")
        self.assertEqual(peer.parse_packet(raw), (peer.SNAPSHOT_REQUEST, 1, b"\xa0"))

    def test_cobs_boundaries(self):
        for size in [0, 1, 253, 254, 255, 508, 1024, 1038]:
            for value in [0, 1, 255]:
                original = bytes([value]) * size
                encoded = peer.cobs_encode(original)
                self.assertNotIn(0, encoded)
                self.assertEqual(peer.cobs_decode(encoded), original)
        for malformed in [b"\0", b"\x03\x01", b"\x02\0"]:
            with self.assertRaises(ValueError):
                peer.cobs_decode(malformed)

    def test_reject_corruption_and_header(self):
        raw = bytearray(peer.raw_packet(peer.TELEMETRY, 8, b"\xa0"))
        raw[10] ^= 1
        with self.assertRaises(ValueError):
            peer.parse_packet(raw)
        raw[0] = 2
        raw[-4:] = struct.pack("<I", zlib.crc32(raw[:-4]))
        with self.assertRaises(ValueError):
            peer.parse_packet(raw)
        with self.assertRaises(ValueError):
            peer.raw_packet(peer.TELEMETRY, 1, bytes(1025))

    def test_cbor_integer_types_and_map_keys(self):
        self.assertEqual(peer.cbor({1001: 24700}).hex(), "a11903e919607c")
        self.assertEqual(peer.cbor([-1, -25, True, False, "V"]).hex(),
                         "85203818f5f46156")

    def test_ble_fragments(self):
        raw = peer.raw_packet(peer.TELEMETRY, 9, bytes(1024))
        for mtu in [23, 247]:
            fragments = peer.fragments(raw, 65535, mtu)
            assembled = bytearray()
            for f in fragments:
                self.assertLessEqual(len(f), mtu - 3)
                identifier, total, offset = struct.unpack_from("<HHH", f)
                self.assertEqual((identifier, total, offset), (65535, len(raw), len(assembled)))
                assembled.extend(f[6:])
            self.assertEqual(assembled, raw)


if __name__ == "__main__":
    unittest.main()
