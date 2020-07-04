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
#ifndef Turnouts_h
#define Turnouts_h

#include <Arduino.h>
#include "DCC.h"

const byte STATUS_ACTIVE=0x80; // Flag as activated
const byte STATUS_PWM=0x40; // Flag as a PWM turnout
const byte STATUS_PWMPIN=0x3F; // PWM  pin 0-63

struct TurnoutData {
   int id;
   uint8_t tStatus; // has STATUS_ACTIVE, STATUS_PWM, STATUS_PWMPIN  
   union {uint8_t subAddress; char moveAngle;}; //DCC  sub addrerss or PWM difference from inactiveAngle  
   union {int address; int inactiveAngle;}; // DCC address or PWM servo angle 
};

struct Turnout{
  static Turnout *firstTurnout;
  TurnoutData data;
  Turnout *nextTurnout;
  static  bool activate(int n, bool state);
  static Turnout* get(int);
  static bool remove(int);
  static void load();
  static void store();
  static Turnout *create(int id , int address , int subAddress);
  static Turnout *create(int id , byte pin , int activeAngle, int inactiveAngle);
  static Turnout *create(int id);
  static void show(Print & stream, int n);
  static bool showAll(Print & stream);
  virtual void activate(bool state);
}; // Turnout
  
#endif
