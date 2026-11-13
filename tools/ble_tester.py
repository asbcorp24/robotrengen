"""Desktop BLE test client for the ESP32 bridge and Blue Pill simulator."""

import asyncio
import json
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

from tools.stm32_simulator import (
    COMMAND, COMMAND_RESULT, ERRORS, HELLO, MAX_RAW, SCHEMA, SCHEMA_REQUEST,
    SNAPSHOT_REQUEST, STOP, TELEMETRY, VERSION, fragments, json_payload,
    parse_packet, raw_packet,
)

SERVICE = "82b10001-95c4-4f26-a9db-96ef80e73210"
RX = "82b10002-95c4-4f26-a9db-96ef80e73210"
TX = "82b10003-95c4-4f26-a9db-96ef80e73210"
INFO = "82b10004-95c4-4f26-a9db-96ef80e73210"
ERROR_EVENT = 0x12


class Notifications:
    def __init__(self):
        self.data = bytearray()
        self.message = None
        self.started = 0.0

    def feed(self, fragment, now):
        if not 7 <= len(fragment) <= 244:
            self.message = None
            raise ValueError("Неверная длина BLE-фрагмента")
        message_id, total, offset = struct.unpack_from("<HHH", fragment)
        if offset == 0:
            self.message = (message_id, total)
            self.data.clear()
            self.started = now
        if (not 14 <= total <= MAX_RAW or self.message != (message_id, total)
                or offset != len(self.data) or now - self.started > 1.0
                or offset + len(fragment) - 6 > total):
            self.message = None
            raise ValueError("Нарушена последовательность BLE-фрагментов")
        self.data.extend(fragment[6:])
        if len(self.data) != total:
            return None
        self.message = None
        kind, sequence, encoded = parse_packet(self.data)
        value = json.loads(encoded.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("Ожидался объект JSON")
        return kind, sequence, value


class Bridge:
    def __init__(self, scanner, client_factory, emit):
        self.scanner = scanner
        self.client_factory = client_factory
        self.emit = emit
        self.client = None
        self.sequence = 0
        self.write_lock = asyncio.Lock()

    async def scan(self):
        devices = await self.scanner.discover(timeout=5, return_adv=True)
        matches = []
        for device, advertisement in devices.values():
            name = advertisement.local_name or device.name or "Crawler"
            if SERVICE in advertisement.service_uuids or name.startswith("Crawler-"):
                matches.append((f"{name}  [{device.address}]", device))
        return sorted(matches, key=lambda match: match[0])

    async def connect(self, device):
        def disconnected(client):
            if self.client is client:
                self.client = None
                self.emit("disconnected", None)

        client = self.client_factory(device, disconnected_callback=disconnected, pair=True, timeout=30)
        notifications = Notifications()

        def receive(_sender, data):
            if self.client is not client:
                return
            try:
                packet = notifications.feed(data, time.monotonic())
                if packet is not None:
                    self.emit("packet", packet)
            except ValueError as error:
                self.emit("log", f"Приём: {error}")

        self.client = client
        try:
            await asyncio.wait_for(client.connect(), timeout=45)
            info = json.loads(await asyncio.wait_for(client.read_gatt_char(INFO), timeout=10))
            if not isinstance(info, dict) or info.get("protocol") != VERSION:
                raise ValueError(f"Требуется прошивка ESP32 с BLE-протоколом v{VERSION}")
            await asyncio.wait_for(client.start_notify(TX, receive), timeout=10)
            self.emit("log", f"BLE v{VERSION}, MTU {client.mtu_size}. Уведомления включены.")
            if self.client is not client or not client.is_connected:
                raise ConnectionError("ESP32 отключилась при подключении")
        except (Exception, asyncio.CancelledError):
            await self.disconnect()
            raise

    async def disconnect(self):
        client, self.client = self.client, None
        if client is not None and client.is_connected:
            await asyncio.wait_for(client.disconnect(), timeout=5)

    async def send(self, kind, value):
        async with self.write_lock:
            client = self.client
            if client is None or not client.is_connected:
                raise ConnectionError("Подключись к ESP32")
            self.sequence = (self.sequence + 1) & 0xffffffff
            sequence = self.sequence
            mtu = max(23, min(client.mtu_size, 247))
            packet = raw_packet(kind, sequence, json_payload(value))
            for fragment in fragments(packet, message_id=sequence & 0xffff, mtu=mtu):
                await asyncio.wait_for(client.write_gatt_char(RX, fragment, response=True), timeout=5)
            return sequence


class App:
    def __init__(self, root, scanner, client_factory):
        self.root = root
        self.events = queue.SimpleQueue()
        self.loop = asyncio.new_event_loop()
        self.bridge = Bridge(scanner, client_factory, lambda kind, data: self.events.put((kind, data)))
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()
        self.future = None
        self.devices = []
        self.connected = False
        self.closing = False
        self.fields = {}
        self.values = {}
        self.value_times = {}
        self.source_boots = {}
        self.simulated_sources = {}
        self.last_refresh = 0.0
        self.total_packets = 0
        self.last_telemetry = None

        root.title("Crawler — проверка BLE")
        root.geometry("760x570")
        root.minsize(620, 480)
        panel = ttk.Frame(root, padding=12)
        panel.pack(fill="both", expand=True)

        connection = ttk.Frame(panel)
        connection.pack(fill="x")
        self.scan_button = ttk.Button(connection, text="Поиск", command=self.scan)
        self.scan_button.pack(side="left")
        self.device = ttk.Combobox(connection, state="readonly")
        self.device.pack(side="left", fill="x", expand=True, padx=8)
        self.connect_button = ttk.Button(connection, text="Подключить", command=self.toggle_connection)
        self.connect_button.pack(side="left")

        self.status = tk.StringVar(value="Включи Bluetooth и нажми «Поиск».")
        self.traffic = tk.StringVar(value="Пакеты: 0")
        ttk.Label(panel, textvariable=self.status, wraplength=590).pack(anchor="w", pady=(8, 2))
        ttk.Label(panel, textvariable=self.traffic).pack(anchor="w")

        table_frame = ttk.Frame(panel)
        table_frame.pack(fill="both", expand=True, pady=8)
        self.table = ttk.Treeview(table_frame, columns=("value", "unit"), height=8)
        self.table.heading("#0", text="Параметр")
        self.table.heading("value", text="Значение")
        self.table.heading("unit", text="Ед.")
        self.table.column("value", width=145)
        self.table.column("unit", width=75, stretch=False)
        self.table.column("#0", width=430, minwidth=260)
        scrollbar = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        scrollbar.pack(side="right", fill="y")
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.pack(side="left", fill="both", expand=True)
        self.errors = tk.StringVar(value="Ошибки: ожидание снимка")
        ttk.Label(panel, textvariable=self.errors, wraplength=590).pack(anchor="w")

        controls = ttk.LabelFrame(panel, text="Симулятор Blue Pill", padding=6)
        controls.pack(fill="x", pady=8)
        self.command_buttons = []
        for label, kind, value in (
            ("Снимок", SNAPSHOT_REQUEST, {}),
            ("Старт", COMMAND, {"op": "start"}),
            ("Стоп", STOP, {}),
            ("Вызвать ошибку", COMMAND, {"op": "raise_fault"}),
            ("Снять ошибку", COMMAND, {"op": "clear_fault"}),
        ):
            button = ttk.Button(controls, text=label,
                                command=lambda k=kind, v=value, title=label: self.send(k, v, title))
            button.pack(side="left", padx=2)
            self.command_buttons.append(button)

        self.log_view = tk.Text(panel, height=7, wrap="word", state="disabled")
        self.log_view.pack(fill="x")
        self.update_buttons()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(80, self.poll)

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def submit(self, action, coroutine):
        self.future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)
        self.future.add_done_callback(lambda future: self.events.put(("done", (action, future))))
        self.update_buttons()

    def update_buttons(self):
        idle = self.future is None and not self.closing
        self.scan_button.configure(state="normal" if idle and not self.connected else "disabled")
        self.device.configure(state="readonly" if idle and not self.connected else "disabled")
        self.connect_button.configure(text="Отключить" if self.connected else "Подключить",
                                      state="normal" if idle and (self.connected or self.devices) else "disabled")
        for button in self.command_buttons:
            button.configure(state="normal" if idle and self.connected else "disabled")

    def scan(self):
        self.status.set("Поиск Crawler…")
        self.submit("scan", self.bridge.scan())

    def toggle_connection(self):
        if self.connected:
            self.submit("disconnect", self.bridge.disconnect())
            return
        selected = self.device.current()
        if selected < 0:
            return
        self.fields.clear()
        self.values.clear()
        self.value_times.clear()
        self.source_boots.clear()
        self.simulated_sources.clear()
        self.update_table()
        self.total_packets = 0
        self.last_telemetry = None
        self.errors.set("Ошибки: ожидание снимка")
        self.status.set("Подключение. Подтверди сопряжение, если Windows попросит.")
        self.submit("connect", self.bridge.connect(self.devices[selected][1]))

    def send(self, kind, value, label):
        self.submit(label, self.bridge.send(kind, value))

    async def request_initial_data(self):
        for kind in (HELLO, SCHEMA_REQUEST, SNAPSHOT_REQUEST):
            await self.bridge.send(kind, {})

    def finish(self, action, future):
        if future is not self.future:
            return
        self.future = None
        try:
            result = future.result()
            if action == "scan":
                self.devices = result
                self.device.configure(values=[label for label, _ in result])
                self.device.set("")
                if result:
                    self.device.current(0)
                self.status.set("Выбери устройство." if result else
                                "Crawler не найден. Отключи телефон от ESP32 и повтори поиск.")
            elif action == "connect":
                self.connected = True
                self.status.set("Подключено. Ожидание пакетов от Blue Pill.")
                self.submit("initial", self.request_initial_data())
            elif action == "disconnect":
                self.mark_disconnected()
            elif action != "initial":
                self.log(f"Отправлено: {action} (seq={result})")
        except Exception as error:
            if action in {"connect", "disconnect"}:
                self.connected = False
            message = str(error) or type(error).__name__
            self.status.set(f"Ошибка BLE: {message}")
            self.log(f"{action}: {message}")
        self.update_buttons()

    def mark_disconnected(self):
        self.connected = False
        self.last_telemetry = None
        self.values.clear()
        self.value_times.clear()
        self.update_table()
        self.errors.set("Ошибки: нет связи")
        self.status.set("Отключено.")
        self.update_buttons()

    def update_table(self):
        rows = self.table.get_children()
        if rows:
            self.table.delete(*rows)
        for key in dict.fromkeys((*self.fields, *self.values)):
            field = self.fields.get(key, {})
            value = self.values.get(key)
            display = "—" if key not in self.values or value is None else (
                value if isinstance(value, str) else json.dumps(value, ensure_ascii=False))
            source, _, raw_key = key.partition("/")
            if key in self.value_times and time.monotonic() - self.value_times[key] > 3:
                display = f"{display} (устарело)"
            label = field.get("name", raw_key)
            simulation = " • имитация" if self.simulated_sources.get(source) else ""
            self.table.insert("", "end", text=f"{source.upper()}{simulation} / {label}",
                              values=(display, field.get("unit", "")))

    def show_packet(self, kind, sequence, value):
        self.total_packets += 1
        source = value.get("source", "stm32")
        if not isinstance(source, str):
            return
        if kind == TELEMETRY and isinstance(value.get("values"), dict):
            boot = value.get("boot_id")
            if source in self.source_boots and self.source_boots[source] != boot:
                for key in list(self.values):
                    if key.startswith(source + "/"):
                        self.values.pop(key, None)
                        self.value_times.pop(key, None)
            self.source_boots[source] = boot
            self.simulated_sources[source] = value.get("simulated", False)
            now = time.monotonic()
            for key, reading in value["values"].items():
                qualified = f"{source}/{key}"
                self.values[qualified] = reading
                self.value_times[qualified] = now
            self.last_telemetry = now
            self.status.set("Подключено. Телеметрия поступает.")
            self.update_table()
        elif kind == SCHEMA and isinstance(value.get("fields"), list):
            for field in value["fields"]:
                if isinstance(field, dict) and isinstance(field.get("key"), str):
                    self.fields[f"{source}/{field['key']}"] = field
            self.update_table()
        elif kind == ERRORS and isinstance(value.get("active"), list):
            active = value["active"]
            labels = [f"{error.get('source', '?')} #{error.get('code', '?')}"
                      for error in active if isinstance(error, dict)]
            self.errors.set("Ошибки: " + (", ".join(labels) if labels else "нет активных"))
        elif kind in (HELLO, COMMAND_RESULT, ERROR_EVENT):
            label = {HELLO: "Hello", COMMAND_RESULT: "Ответ", ERROR_EVENT: "Событие"}[kind]
            self.log(f"{label} (seq={sequence}): {json.dumps(value, ensure_ascii=False)}")

    def log(self, line):
        self.log_view.configure(state="normal")
        self.log_view.insert("end", line + "\n")
        if int(self.log_view.index("end-1c").split(".")[0]) > 201:
            self.log_view.delete("1.0", "2.0")
        self.log_view.see("end")
        self.log_view.configure(state="disabled")

    def poll(self):
        if self.closing:
            return
        for _ in range(200):
            try:
                event, data = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "done":
                self.finish(*data)
            elif event == "packet":
                self.show_packet(*data)
            elif event == "disconnected":
                self.mark_disconnected()
            elif event == "log":
                self.log(data)
        age = "—" if self.last_telemetry is None else f"{time.monotonic() - self.last_telemetry:.1f} с назад"
        self.traffic.set(f"Пакеты: {self.total_packets}   •   Последняя телеметрия: {age}")
        if time.monotonic() - self.last_refresh >= 1:
            self.last_refresh = time.monotonic()
            self.update_table()
        self.root.after(80, self.poll)

    def close(self):
        self.closing = True
        self.update_buttons()

        async def shutdown():
            pending = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
            for task in pending:
                task.cancel()
            await asyncio.gather(*pending, return_exceptions=True)
            await self.bridge.disconnect()

        future = asyncio.run_coroutine_threadsafe(shutdown(), self.loop)

        def wait():
            if future.done():
                self.loop.call_soon_threadsafe(self.loop.stop)
                self.root.destroy()
            else:
                self.root.after(80, wait)
        wait()


def main():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        raise SystemExit('Установи Bleak: python -m pip install "bleak>=1.1,<4"') from None
    root = tk.Tk()
    App(root, BleakScanner, BleakClient)
    root.mainloop()


if __name__ == "__main__":
    main()
