#ifndef EEStore_h
#define EEStore_h

#include <Arduino.h>

#if defined(ARDUINO_ARCH_SAMD)
#include <SparkFun_External_EEPROM.h>
extern ExternalEEPROM EEPROM;
#else
#include <EEPROM.h>
#endif

#define EESTORE_ID "DCC++"

struct EEStoreData{
  char id[sizeof(EESTORE_ID)];
  uint16_t nTurnouts;
  uint16_t nSensors;
  uint16_t nOutputs;
};

struct EEStore{
  static EEStore *eeStore;
  EEStoreData data;
  static int eeAddress;
  static void init();
  static void reset();
  static int pointer();
  static void advance(int);
  static void store();
  static void clear();
  static void dump(int);
};

#endif
