#include "../EEStoreFormat.h"
#include <cassert>
#include <cstdint>
#include <cstring>

int main() {
  EEStoreData stored = blankEEStoreData();
  stored.nTurnouts = 3;
  stored.nSensors = 5;
  stored.nOutputs = 7;

  std::uint8_t bytes[sizeof(stored)]{};
  std::memcpy(bytes, &stored, sizeof(stored));
  EEStoreData loaded{};
  std::memcpy(&loaded, bytes, sizeof(loaded));
  assert(isLegacyEEStoreData(loaded));
  assert(loaded.nTurnouts == 3 && loaded.nSensors == 5 && loaded.nOutputs == 7);

  bytes[0] = 'X';
  std::memcpy(&loaded, bytes, sizeof(loaded));
  assert(!isLegacyEEStoreData(loaded));
}
