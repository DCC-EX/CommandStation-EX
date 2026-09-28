/* Persistent settings format shared by firmware and host tests. */
#ifndef EEStoreFormat_h
#define EEStoreFormat_h

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define EESTORE_ID "DCC++1"

struct EEStoreData {
  char id[sizeof(EESTORE_ID)];
  uint16_t nTurnouts;
  uint16_t nSensors;
  uint16_t nOutputs;
};

static_assert(offsetof(EEStoreData, id) == 0, "legacy header must start with id");
static_assert(sizeof(EEStoreData) == sizeof(EESTORE_ID) + 6,
              "legacy header layout changed");

inline bool isLegacyEEStoreData(const EEStoreData &data) {
  return memcmp(data.id, EESTORE_ID, sizeof(EESTORE_ID)) == 0;
}

inline EEStoreData blankEEStoreData() {
  EEStoreData data{};
  memcpy(data.id, EESTORE_ID, sizeof(EESTORE_ID));
  return data;
}

#endif
