#include "BridgeProtocol.h"

#include <string.h>

namespace wire {
uint16_t read16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
uint32_t read32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
void write16(uint8_t* p, uint16_t v) {
  p[0] = v & 0xff;
  p[1] = v >> 8;
}
void write32(uint8_t* p, uint32_t v) {
  for (size_t i = 0; i < 4; ++i) p[i] = (v >> (8 * i)) & 0xff;
}
uint32_t crc32(const uint8_t* data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
  }
  return crc ^ 0xffffffffu;
}
size_t encodeRaw(const Packet& packet, uint8_t* out, size_t capacity) {
  const size_t size = kHeaderSize + packet.size + kCrcSize;
  if (packet.size > kMaxPayload || capacity < size) return 0;
  out[0] = kVersion;
  out[1] = static_cast<uint8_t>(packet.type);
  write16(out + 2, 0);  // Reserved flags; v1 requires zero.
  write16(out + 4, packet.size);
  write32(out + 6, packet.sequence);
  memcpy(out + kHeaderSize, packet.payload, packet.size);
  write32(out + size - kCrcSize, crc32(out, size - kCrcSize));
  return size;
}
bool decodeRaw(const uint8_t* data, size_t size, Packet& packet) {
  if (size < kHeaderSize + kCrcSize || size > kMaxRaw) return false;
  if (data[0] != kVersion || read16(data + 2) != 0) return false;
  const uint16_t payloadSize = read16(data + 4);
  if (payloadSize > kMaxPayload || size != kHeaderSize + payloadSize + kCrcSize)
    return false;
  if (read32(data + size - kCrcSize) != crc32(data, size - kCrcSize)) return false;
  packet.type = static_cast<Type>(data[1]);
  packet.sequence = read32(data + 6);
  packet.size = payloadSize;
  memcpy(packet.payload, data + kHeaderSize, payloadSize);
  return true;
}
size_t cobsEncode(const uint8_t* data, size_t size, uint8_t* out, size_t capacity) {
  if (!capacity) return 0;
  size_t write = 1, codeIndex = 0;
  uint8_t code = 1;
  for (size_t i = 0; i < size; ++i) {
    if (data[i] == 0) {
      out[codeIndex] = code;
      codeIndex = write;
      if (write >= capacity) return 0;
      ++write;
      code = 1;
    } else {
      if (write >= capacity) return 0;
      out[write++] = data[i];
      if (++code == 0xff) {
        out[codeIndex] = code;
        codeIndex = write;
        if (write >= capacity) return 0;
        ++write;
        code = 1;
      }
    }
  }
  out[codeIndex] = code;
  return write;
}
size_t cobsDecode(const uint8_t* data, size_t size, uint8_t* out, size_t capacity) {
  size_t read = 0, write = 0;
  if (!size) return 0;
  while (read < size) {
    const uint8_t code = data[read++];
    if (!code || static_cast<size_t>(code - 1) > size - read) return 0;
    for (unsigned i = 1; i < code; ++i) {
      if (write >= capacity || data[read] == 0) return 0;
      out[write++] = data[read++];
    }
    if (code != 0xff && read < size) {
      if (write >= capacity) return 0;
      out[write++] = 0;
    }
  }
  return write;
}
bool allowedFromPhone(Type t) {
  return t == Type::Hello || t == Type::SchemaRequest || t == Type::SnapshotRequest ||
         t == Type::Command || t == Type::Stop || t == Type::EventAck;
}
bool allowedFromController(Type t) {
  return t == Type::Hello || t == Type::Schema || t == Type::Telemetry ||
         t == Type::ErrorsSnapshot || t == Type::ErrorEvent || t == Type::CommandResult;
}
void UartDecoder::reset() {
  used_ = 0;
  discarding_ = false;
}
void UartDecoder::expire(uint32_t now, uint32_t timeoutMs) {
  if (used_ && static_cast<uint32_t>(now - lastByteMs_) > timeoutMs) {
    used_ = 0;
    discarding_ = true;  // Consume through delimiter; do not accept a tail as a frame.
  }
}
FeedResult UartDecoder::feed(uint8_t byte, uint32_t now, Packet& packet) {
  lastByteMs_ = now;
  if (byte == 0) {
    if (discarding_) {
      reset();
      return FeedResult::Rejected;
    }
    if (!used_) return FeedResult::None;
    const size_t size = cobsDecode(encoded_, used_, raw_, sizeof(raw_));
    used_ = 0;
    return size && decodeRaw(raw_, size, packet) ? FeedResult::Complete : FeedResult::Rejected;
  }
  if (discarding_) return FeedResult::None;
  if (used_ == sizeof(encoded_)) {
    used_ = 0;
    discarding_ = true;
  } else {
    encoded_[used_++] = byte;
  }
  return FeedResult::None;
}
void Reassembler::reset() {
  active_ = false;
  used_ = total_ = 0;
}
void Reassembler::expire(uint32_t now, uint32_t timeoutMs) {
  if (active_ && static_cast<uint32_t>(now - startedMs_) > timeoutMs) reset();
}
FeedResult Reassembler::feed(const uint8_t* data, size_t size, uint32_t now, Packet& packet) {
  if (size <= kFragmentHeader || size > kMaxBleValue) {
    reset();
    return FeedResult::Rejected;
  }
  const uint16_t id = read16(data), total = read16(data + 2), offset = read16(data + 4);
  const size_t bytes = size - kFragmentHeader;
  if (total < kHeaderSize + kCrcSize || total > kMaxRaw || offset > total ||
      bytes > static_cast<size_t>(total - offset)) {
    reset();
    return FeedResult::Rejected;
  }
  if (offset == 0) {
    reset();
    active_ = true;
    id_ = id;
    total_ = total;
    startedMs_ = now;
  }
  if (!active_ || id != id_ || total != total_ || offset != used_) {
    reset();
    return FeedResult::Rejected;
  }
  memcpy(raw_ + used_, data + kFragmentHeader, bytes);
  used_ += bytes;
  if (used_ != total_) return FeedResult::None;
  const bool valid = decodeRaw(raw_, used_, packet);
  reset();
  return valid ? FeedResult::Complete : FeedResult::Rejected;
}
}  // namespace wire
