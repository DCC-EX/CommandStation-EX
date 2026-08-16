#include <cassert>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>

template <typename T>
constexpr T gpioPinMask(unsigned pin) {
  return T(1) << pin;
}

bool productionUsesTemplateMask() {
  const char *paths[] = {"IO_GPIOBase.h", "../IO_GPIOBase.h"};
  for (const char *path : paths) {
    std::ifstream source(path);
    if (!source) continue;
    std::string contents((std::istreambuf_iterator<char>(source)),
                         std::istreambuf_iterator<char>());
    return contents.find("T mask = T(1) << pin;") != std::string::npos;
  }
  return false;
}

static_assert(gpioPinMask<std::uint16_t>(15) == UINT16_C(0x8000),
              "16-pin GPIO masks must retain the highest pin bit");
static_assert(gpioPinMask<std::uint32_t>(31) == UINT32_C(0x80000000),
              "32-pin GPIO masks must retain the highest pin bit");
static_assert(gpioPinMask<std::uint64_t>(63) == UINT64_C(0x8000000000000000),
              "64-pin GPIO masks must retain the highest pin bit");

int main() {
  assert(productionUsesTemplateMask());
  assert(gpioPinMask<std::uint16_t>(15) == UINT16_C(0x8000));
  assert(gpioPinMask<std::uint32_t>(31) == UINT32_C(0x80000000));
  assert(gpioPinMask<std::uint64_t>(63) == UINT64_C(0x8000000000000000));
}
