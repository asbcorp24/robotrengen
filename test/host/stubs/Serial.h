#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
constexpr int PA_9 = 9, PA_10 = 10, SERIAL_8N1 = 0;
class Uart {
 public:
  std::vector<uint8_t> bytes;
  Uart(int, int) {}
  void begin(int, int) {}
  int available() const { return 0; }
  int read() const { return -1; }
  size_t write(uint8_t value) { bytes.push_back(value); return 1; }
  size_t write(const uint8_t* data, size_t size) {
    bytes.insert(bytes.end(), data, data + size);
    return size;
  }
};
