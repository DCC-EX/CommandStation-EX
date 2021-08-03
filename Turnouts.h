/*
 *  © 2020, Chris Harlow. All rights reserved.
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

/* 
 * Turnout data is stored in a structure whose length depends on the
 * type of turnout.  There is a common header of 3 bytes, followed by
 * 2 bytes for DCC turnout, 5 bytes for servo turnout, 2 bytes for a
 * VPIN turnout, or zero bytes for an LCN turnout.
 * The variable length allows the limited space in EEPROM to be used effectively.
 */

#ifndef Turnouts_h
#define Turnouts_h

#include <Arduino.h>
#include "DCC.h"
#include "LCN.h"
#include "IODevice.h"

const byte STATUS_ACTIVE=0x80; // Flag as activated in tStatus field
const byte STATUS_TYPE = 0x7f;  // Mask for turnout type in tStatus field

//  The struct 'header' is used to determine the length of the
//  overlaid data so must be at least as long as the anonymous fields it
//  is overlaid with.
struct TurnoutData {
  // Header common to all turnouts
  union {
    struct {
      int id;
      uint8_t tStatus;
      uint8_t size;
    } header;

    struct {
      int id;
      union {
        uint8_t tStatus;
        struct {
          uint8_t active: 1;
          uint8_t type: 5;
          uint8_t :2;
        };
      };
      uint8_t size;  // set to actual total length of used structure
    };
  };
  // Turnout-type-specific structure elements, different length depending
  //  on turnout type.  This allows the data to be packed efficiently 
  //  in the EEPROM.
  union {
    struct {
      // DCC address (Address in bits 15-2, subaddress in bits 1-0
      uint16_t address; // CS currently supports linear address 1-2048
        // That's DCC accessory address 1-512 and subaddress 0-3.
    } dccAccessoryData;

    struct {
      VPIN vpin;
      uint16_t activePosition : 12;  // 0-4095
      uint16_t inactivePosition : 12; // 0-4095
      uint8_t profile;
    } servoData;

    struct {
    } lcnData;

    struct {
      VPIN vpin;
    } vpinData;
  };
};

class Turnout {
public:
  static Turnout *firstTurnout;
  static int turnoutlistHash;
  TurnoutData data;
  Turnout *nextTurnout;
  static bool activate(int n, bool state);
  static Turnout* get(int);
  static bool remove(int);
  static bool isActive(int);
  static void setActive(int n, bool state);
  static void load();
  static void store();
  static Turnout *createServo(int id , VPIN vpin , uint16_t activeAngle, uint16_t inactiveAngle, uint8_t profile=1, uint8_t initialState=0);
  static Turnout *createVpin(int id, VPIN vpin, uint8_t initialState=0);
  static Turnout *createDCC(int id, uint16_t address, uint8_t subAddress);
  static Turnout *createLCN(int id, uint8_t initialState=0);
  static Turnout *create(int id, int params, int16_t p[]);
  static Turnout *create(int id);
  void activate(bool state);
  void setActive(bool state);
  bool isActive();
  static void printAll(Print *);
  void print(Print *stream);
#ifdef EESTOREDEBUG
  static void print(Turnout *tt);
#endif
private:
  int num;  // EEPROM address of tStatus in TurnoutData struct, or zero if not stored.
}; // Turnout
  
#endif
