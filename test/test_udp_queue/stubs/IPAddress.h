#pragma once
#include <cstdint>
class IPAddress {
public:
  explicit IPAddress(uint32_t value = 0) : value(value) {}
  bool operator==(const IPAddress &other) const { return value == other.value; }
private:
  uint32_t value;
};
