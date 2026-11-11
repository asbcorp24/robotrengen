#pragma once

#include <stdint.h>

namespace config {
// ESP32 TX -> STM32 RX; ESP32 RX <- STM32 TX. Common GND, 3.3 V logic.
// Change these pins to match the actual PCB before connecting the controller.
constexpr int kUartRxPin = 18;
constexpr int kUartTxPin = 17;
constexpr uint32_t kUartBaud = 115200;
constexpr uint32_t kCommandMaxAgeMs = 250;
constexpr uint32_t kUartFrameTimeoutMs = 250;
constexpr uint32_t kBleAssemblyTimeoutMs = 1000;
constexpr uint32_t kBleSendTimeoutMs = 1000;
constexpr uint32_t kNotifyIntervalMs = 10;
constexpr uint16_t kBleMtu = 247;
constexpr uint32_t kStatsIntervalMs = 5000;

constexpr char kServiceUuid[] = "82b10001-95c4-4f26-a9db-96ef80e73210";
constexpr char kRxUuid[] = "82b10002-95c4-4f26-a9db-96ef80e73210";
constexpr char kTxUuid[] = "82b10003-95c4-4f26-a9db-96ef80e73210";
constexpr char kInfoUuid[] = "82b10004-95c4-4f26-a9db-96ef80e73210";
}  // namespace config
