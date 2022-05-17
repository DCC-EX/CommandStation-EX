/*
 *  © 2021 Neil McKechnie
 *  © 2021 Fred Decker
 *  © 2020-2021 Harald Barth
 *  © 2020 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef DISABLE_EEPROM
#ifndef EEStore_h
#define EEStore_h

#include <Arduino.h>

#if defined(ARDUINO_ARCH_SAMC)
#include <SparkFun_External_EEPROM.h>
extern ExternalEEPROM EEPROM;
#else
#include <EEPROM.h>
#endif

#define EESTORE_ID "DCC++1"

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
#endif // DISABLE_EEPROM
