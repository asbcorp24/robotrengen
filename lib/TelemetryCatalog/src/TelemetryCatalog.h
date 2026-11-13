#pragma once

#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>

namespace catalog {
struct Field {
  const char* key;
  const char* name;
  const char* type;
  const char* unit;
  const char* group;
  uint16_t periodMs;
};

constexpr uint32_t kRevision = 2;
constexpr size_t kFieldsPerPage = 2;

const Field kStm32Fields[] = {
    {"distance", "Расстояние от исходной позиции", "number", "m", "motion", 200},
    {"odometer_km", "Общий пробег", "uint32", "km", "battery", 1000},
    {"moving", "Движение", "bool", "", "motion", 200},
    {"crawler_state", "Состояние кроулера", "string", "", "motion", 200},
    {"drive_battery_voltage", "Напряжение ходового АКБ", "number", "V", "battery", 1000},
    {"drive_battery_current", "Ток ходового АКБ", "number", "A", "battery", 1000},
    {"drive_battery_temperature", "Температура ходового АКБ", "number", "degC", "battery", 1000},
    {"drive_battery_soc", "Заряд ходового АКБ", "number", "%", "battery", 1000},
    {"xray_battery_voltage", "Напряжение рентгеновского АКБ", "number", "V", "battery", 1000},
    {"xray_battery_current", "Ток рентгеновского АКБ", "number", "A", "battery", 1000},
    {"xray_battery_temperature", "Температура рентгеновского АКБ", "number", "degC", "battery", 1000},
    {"xray_battery_soc", "Заряд рентгеновского АКБ", "number", "%", "battery", 1000},
    {"xray_state", "Состояние рентгеновского аппарата", "string", "", "xray", 200},
    {"xray_voltage", "Напряжение аппарата", "number", "kV", "xray", 200},
    {"xray_current", "Ток аппарата", "number", "mA", "xray", 200},
    {"exposure_remaining_s", "Осталось до конца экспозиции", "uint32", "s", "xray", 200},
    {"exposure_delay_remaining_s", "Задержка до экспозиции", "uint32", "s", "xray", 200},
    {"xray_error_mask", "Маска ошибок аппарата 1–16", "uint16", "", "xray", 200},
    {"sensor_start_triggered", "Датчик начала трубы", "bool", "", "motion", 200},
    {"sensor_end_triggered", "Датчик конца трубы", "bool", "", "motion", 200},
    {"sensor_water_triggered", "Датчик воды", "bool", "", "motion", 200},
    {"sensor_obstacle_triggered", "Датчик препятствия", "bool", "", "motion", 200},
    {"roll", "Крен", "number", "deg", "motion", 200},
    {"pitch", "Тангаж", "number", "deg", "motion", 200},
    {"brake_force_pct", "Измеренное усилие тормоза", "number", "%", "motion", 200},
    {"brake_command_pct", "Задание тормоза", "number", "%", "motion", 200},
    {"magnetic_signal_pct", "Сигнал магнитных приёмников", "array", "%", "diagnostics", 1000},
    {"imu_health", "Самодиагностика ASM330LHHX", "string", "", "diagnostics", 1000},
    {"tof_health", "Самодиагностика VL53L4ED", "string", "", "diagnostics", 1000},
    {"analog_health", "Самодиагностика аналоговых каналов", "string", "", "diagnostics", 1000},
    {"sensor_start_health", "Исправность датчика начала", "string", "", "diagnostics", 1000},
    {"sensor_end_health", "Исправность датчика конца", "string", "", "diagnostics", 1000},
    {"sensor_water_health", "Исправность датчика воды", "string", "", "diagnostics", 1000},
    {"sensor_obstacle_health", "Исправность датчика препятствия", "string", "", "diagnostics", 1000},
};

const Field kEsp32Fields[] = {
    {"uptime_ms", "Время работы ESP32", "uint32", "ms", "bridge", 1000},
    {"heap_free_bytes", "Свободная RAM ESP32", "uint32", "bytes", "bridge", 1000},
    {"heap_min_bytes", "Минимум свободной RAM", "uint32", "bytes", "bridge", 1000},
    {"ble_rssi_dbm", "RSSI на стороне ESP32", "int32", "dBm", "bridge", 1000},
    {"ble_mtu", "Согласованный BLE MTU", "uint16", "bytes", "bridge", 1000},
    {"ble_tx_phy", "PHY передачи ESP32", "string", "", "bridge", 1000},
    {"ble_rx_phy", "PHY приёма ESP32", "string", "", "bridge", 1000},
    {"stm32_link_ok", "Связь UART со STM32", "bool", "", "bridge", 1000},
    {"stm32_last_rx_age_ms", "Возраст пакета STM32", "uint32", "ms", "bridge", 1000},
    {"uart_rx_packets", "Пакеты UART RX", "uint32", "", "bridge", 1000},
    {"uart_bad_packets", "Ошибки UART", "uint32", "", "bridge", 1000},
    {"uart_tx_packets", "Пакеты UART TX", "uint32", "", "bridge", 1000},
    {"ble_rx_packets", "Пакеты BLE RX", "uint32", "", "bridge", 1000},
    {"ble_tx_packets", "Пакеты BLE TX", "uint32", "", "bridge", 1000},
    {"queue_drops", "Потери очередей", "uint32", "", "bridge", 1000},
    {"ingress_drops", "Потери входных фрагментов", "uint32", "", "bridge", 1000},
    {"expired_packets", "Просроченные пакеты", "uint32", "", "bridge", 1000},
    {"notify_failures", "Ошибки отправки BLE", "uint32", "", "bridge", 1000},
};

template <size_t Count>
size_t pageCount(const Field (&)[Count]) {
  return (Count + kFieldsPerPage - 1) / kFieldsPerPage;
}

template <size_t Count>
void schemaPage(JsonDocument& doc, const char* source, const Field (&fields)[Count], size_t page) {
  doc.clear();
  doc["source"] = source;
  doc["revision"] = kRevision;
  doc["part_index"] = page;
  doc["parts_total"] = pageCount(fields);
  JsonArray entries = doc["fields"].to<JsonArray>();
  const size_t end = (page + 1) * kFieldsPerPage < Count ? (page + 1) * kFieldsPerPage : Count;
  for (size_t i = page * kFieldsPerPage; i < end; ++i) {
    const auto& field = fields[i];
    JsonObject entry = entries.add<JsonObject>();
    entry["key"] = field.key;
    entry["name"] = field.name;
    entry["type"] = field.type;
    entry["unit"] = field.unit;
    entry["group"] = field.group;
    entry["period_ms"] = field.periodMs;
    entry["access"] = "read";
  }
}
}  // namespace catalog
