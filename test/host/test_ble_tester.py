import struct
import unittest

from tools.ble_tester import App, Bridge, Notifications, INFO, RX, TX
from tools.stm32_simulator import COMMAND, TELEMETRY, SCHEMA, fragments, json_payload, raw_packet


class NotificationsTests(unittest.TestCase):
    def test_crc_and_fragment_recovery(self):
        value = {"values": {"distance": 120, "moving": True}}
        packet = raw_packet(TELEMETRY, 9, json_payload(value))
        chunks = fragments(packet, message_id=9, mtu=23)
        reader = Notifications()
        self.assertIsNone(reader.feed(chunks[0], 0.0))
        with self.assertRaisesRegex(ValueError, "последовательность"):
            reader.feed(chunks[1], 1.1)

        for chunk in chunks[:-1]:
            self.assertIsNone(reader.feed(chunk, 2.0))
        damaged = bytearray(chunks[-1])
        damaged[-1] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC"):
            reader.feed(damaged, 2.0)

        for chunk in chunks[:-1]:
            self.assertIsNone(reader.feed(chunk, 3.0))
        self.assertEqual(reader.feed(chunks[-1], 3.0), (TELEMETRY, 9, value))

    def test_telemetry_groups_and_sources_are_preserved(self):
        app = App.__new__(App)
        app.total_packets = 0
        app.values, app.fields, app.value_times = {}, {}, {}
        app.source_boots, app.simulated_sources = {}, {}
        app.status = type("Label", (), {"set": lambda *_: None})()
        app.update_table = lambda: None
        app.show_packet(TELEMETRY, 1, {
            "source": "stm32", "boot_id": 1, "simulated": True,
            "values": {"distance": 12.3}})
        app.show_packet(TELEMETRY, 2, {
            "source": "stm32", "boot_id": 1, "values": {"xray_error_mask": 0}})
        app.show_packet(TELEMETRY, 1, {
            "source": "esp32", "boot_id": 2, "values": {"ble_rssi_dbm": -64}})
        self.assertEqual(app.values, {
            "stm32/distance": 12.3, "stm32/xray_error_mask": 0, "esp32/ble_rssi_dbm": -64})
        app.show_packet(SCHEMA, 5, {"source": "esp32", "fields": [{"key": "ble_rssi_dbm", "unit": "dBm"}]})
        self.assertEqual(app.fields["esp32/ble_rssi_dbm"]["unit"], "dBm")
        app.show_packet(TELEMETRY, 1, {
            "source": "stm32", "boot_id": 3, "values": {"distance": 0}})
        self.assertNotIn("stm32/xray_error_mask", app.values)
        self.assertEqual(app.values["esp32/ble_rssi_dbm"], -64)


class BridgeTests(unittest.IsolatedAsyncioTestCase):
    async def test_pair_subscribe_and_write(self):
        events = []
        bridge = Bridge(None, FakeClient, lambda *event: events.append(event))
        await bridge.connect("test")
        client = bridge.client
        self.assertTrue(client.options["pair"])
        self.assertEqual(client.reads, [INFO])
        self.assertEqual(client.notify_uuid, TX)

        seq = await bridge.send(COMMAND, {"op": "start"})
        self.assertEqual(len(client.writes), 2)
        reader = Notifications()
        for uuid, data, response in client.writes:
            self.assertEqual((uuid, response), (RX, True))
            self.assertLessEqual(len(data), 20)
            self.assertEqual(struct.unpack_from("<H", data)[0], seq)
            message = reader.feed(data, 0.0)
        self.assertEqual(message, (COMMAND, seq, {"op": "start"}))

        value = {"values": {"moving": True}}
        for chunk in fragments(raw_packet(TELEMETRY, 11, json_payload(value))):
            client.notify(None, chunk)
        self.assertIn(("packet", (TELEMETRY, 11, value)), events)

        await bridge.disconnect()
        self.assertFalse(client.is_connected)
        self.assertIsNone(bridge.client)


class FakeClient:
    mtu_size = 23

    def __init__(self, device, **options):
        self.options = options
        self.is_connected = False
        self.reads = []
        self.writes = []

    async def connect(self):
        self.is_connected = True

    async def disconnect(self):
        self.is_connected = False

    async def read_gatt_char(self, uuid):
        self.reads.append(uuid)
        return b'{"protocol":2}'

    async def start_notify(self, uuid, callback):
        self.notify_uuid = uuid
        self.notify = callback

    async def write_gatt_char(self, uuid, data, *, response):
        self.writes.append((uuid, data, response))


if __name__ == "__main__":
    unittest.main()
