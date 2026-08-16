#include <cassert>
#include <cstdint>

template <typename T>
constexpr T gpioPinMask(unsigned pin) {
  return T(1) << pin;
}

static_assert(gpioPinMask<std::uint16_t>(15) == UINT16_C(0x8000),
              "16-pin GPIO masks must retain the highest pin bit");
static_assert(gpioPinMask<std::uint32_t>(31) == UINT32_C(0x80000000),
              "32-pin GPIO masks must retain the highest pin bit");
static_assert(gpioPinMask<std::uint64_t>(63) == UINT64_C(0x8000000000000000),
              "64-pin GPIO masks must retain the highest pin bit");

int main() {
  assert(gpioPinMask<std::uint16_t>(15) == UINT16_C(0x8000));
  assert(gpioPinMask<std::uint32_t>(31) == UINT32_C(0x80000000));
  assert(gpioPinMask<std::uint64_t>(63) == UINT64_C(0x8000000000000000));
}
