/*
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
 *  © 2020 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#ifndef Outputs_h
#define Outputs_h

#include <Arduino.h>
#include "IODevice.h"

struct OutputData {
  union {
    uint8_t oStatus;      // (Bit 0=Invert, Bit 1=Set state to default, Bit 2=default state, Bit 7=active)
    struct {
      unsigned int flags : 7; // Bit 0=Invert, Bit 1=Set state to default, Bit 2=default state
      unsigned int : 1;
    };
    struct {
      unsigned int invert : 1;
      unsigned int setDefault : 1;
      unsigned int defaultValue : 1;
      unsigned int: 4;
      unsigned int active : 1;
    };
  };
  uint16_t id;
  VPIN pin; 
};


class Output{
public:
  void activate(uint16_t s);
  bool isActive();
  static Output* get(uint16_t);
  static bool remove(uint16_t);
#ifndef DISABLE_EEPROM
  static void load();
  static void store();
#endif
  static Output *create(uint16_t, VPIN, int, int=0);
  static Output *firstOutput;
  struct OutputData data;
  Output *nextOutput;
  static void printAll(Print *);
private:
  uint16_t num;  // EEPROM address of oStatus in OutputData struct, or zero if not stored.
  
}; // Output
  
#endif
