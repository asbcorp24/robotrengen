#pragma once

#include <stddef.h>
#include <stdint.h>

// No Arduino dependency: shared wire protocol for ESP32, STM32 and host tests.
namespace wire {
constexpr uint8_t kVersion = 2;  // BLE with JSON
constexpr uint8_t kUartVersion = 3;  // UART with CBOR
constexpr size_t kHeaderSize = 10;
constexpr size_t kCrcSize = 4;
constexpr size_t kMaxPayload = 1024;
constexpr size_t kMaxRaw = kHeaderSize + kMaxPayload + kCrcSize;
constexpr size_t kMaxEncoded = kMaxRaw + kMaxRaw / 254 + 1;
constexpr size_t kFragmentHeader = 6;
constexpr size_t kMaxBleValue = 244;

enum class Type : uint8_t {
  Hello = 0x01,
  SchemaRequest = 0x02,
  Schema = 0x03,
  SnapshotRequest = 0x04,
  Telemetry = 0x10,
  ErrorsSnapshot = 0x11,
  ErrorEvent = 0x12,
  Command = 0x20,
  Stop = 0x21,
  CommandResult = 0x22,
  EventAck = 0x23,
  LinkState = 0x40,  // ESP32 -> STM32 only; never a command acknowledgement.
};

struct Packet {
  Type type = Type::Hello;
  uint32_t sequence = 0;
  uint16_t size = 0;
  uint8_t payload[kMaxPayload]{};
};

uint16_t read16(const uint8_t* p);
uint32_t read32(const uint8_t* p);
void write16(uint8_t* p, uint16_t value);
void write32(uint8_t* p, uint32_t value);
uint32_t crc32(const uint8_t* data, size_t size);
size_t encodeRaw(const Packet& packet, uint8_t* out, size_t capacity, uint8_t version = kVersion);
bool decodeRaw(const uint8_t* data, size_t size, Packet& packet, uint8_t version = kVersion);
size_t cobsEncode(const uint8_t* data, size_t size, uint8_t* out, size_t capacity);
size_t cobsDecode(const uint8_t* data, size_t size, uint8_t* out, size_t capacity);
bool allowedFromPhone(Type type);
bool allowedFromController(Type type);

enum class FeedResult { None, Complete, Rejected };

class UartDecoder {
 public:
  FeedResult feed(uint8_t byte, uint32_t now, Packet& packet);
  void reset();
  void expire(uint32_t now, uint32_t timeoutMs);

 private:
  uint8_t encoded_[kMaxEncoded]{};
  uint8_t raw_[kMaxRaw]{};
  size_t used_ = 0;
  bool discarding_ = false;
  uint32_t lastByteMs_ = 0;
};

// One ordered message at a time. Header: message_id:u16, total:u16, offset:u16.
// Offset 0 starts a new message; out-of-order or duplicate fragments abort it.
class Reassembler {
 public:
  FeedResult feed(const uint8_t* data, size_t size, uint32_t now, Packet& packet);
  void reset();
  void expire(uint32_t now, uint32_t timeoutMs);

 private:
  uint8_t raw_[kMaxRaw]{};
  uint16_t id_ = 0;
  uint16_t total_ = 0;
  uint16_t used_ = 0;
  uint32_t startedMs_ = 0;
  bool active_ = false;
};

template <typename T, size_t Capacity>
class Queue {
 public:
  static_assert(Capacity > 0, "Queue capacity must be positive");
  bool push(const T& value) {
    if (count_ == Capacity) return false;
    items_[(head_ + count_) % Capacity] = value;
    ++count_;
    return true;
  }
  bool pop(T& value) {
    if (!count_) return false;
    value = items_[head_];
    head_ = (head_ + 1) % Capacity;
    --count_;
    return true;
  }
  void clear() { head_ = count_ = 0; }
  size_t size() const { return count_; }

 private:
  T items_[Capacity]{};
  size_t head_ = 0;
  size_t count_ = 0;
};
}  // namespace wire
