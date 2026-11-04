/*
 *  © 2023 Peter Cole
 *  © 2026 Chris Harlow
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

#ifndef TURNTABLES_H
#define TURNTABLES_H

#include <Arduino.h>
#include "IODevice.h"
#include "StringFormatter.h"
#include "IO_EXTurntable.h"

enum TurntableType:byte {
  TURNTABLE_EXTT = 0,
  TURNTABLE_DCC = 1,
  TURNTABLE_REMOTE = 2,     // Turntable is on nother node
};


/*************************************************************************************
 * Turntable positions.
 * 
 *************************************************************************************/
struct TurntablePosition {
  uint8_t index;
  union {
    uint16_t dccAddress; // turntable type =DCC
    uint16_t exttData;   // turntable type = EXTT
  };
  uint16_t angle;
  char * description;
  TurntablePosition* next;
};


/*************************************************************************************
 * Turntable - Base class for turntables.
 * 
 *************************************************************************************/

class Turntable {
  public:
    static Turntable* createEXTT(uint16_t id, uint16_t homeAngle, VPIN extVpin, const char * description="") {
      return create(id, TURNTABLE_EXTT, homeAngle, extVpin, description);
    }
    static Turntable* createDCC(uint16_t id, uint16_t homeAngle, const char * description="") {
      return create(id, TURNTABLE_DCC, homeAngle, 0, description);
    }
    static Turntable* createREMOTE(uint16_t id, uint16_t homeAngle, const char * description="") {
      return create(id, TURNTABLE_REMOTE, homeAngle, 0, description);
    }
    
    static void printAll(Print *stream);
    static Turntable *get(uint16_t id);
    static uint16_t getByVpin(VPIN vpin); // used for back reference from HAL driver
    static void shareNodesToCS();
    static bool setPosition(uint16_t id, uint8_t position, byte activity, bool tellNodes=true);
    static bool setPosition(uint16_t id, uint8_t position, ActivityNumber activity=Turn);
    static uint8_t getPosition(uint16_t id);
    static bool isMoving(uint16_t id);
    static void setMoving(uint16_t id, bool isMoving, bool tellNodes=true);
    
    
    static Turntable *first() { return _firstTurntable; }
    Turntable *next() { return _nextTurntable;}
    TurntablePosition * firstPosition() { return _firstPosition; }
    uint16_t getId() { return id; }
    bool isHidden() { return description[0]==0x01; }
    int8_t getPosition() { return position; }
    const char * getDescription() { return description; }
    uint8_t getPositionCount();
    TurntablePosition * addPosition(uint8_t idx, uint16_t value, uint16_t angle, const char * description="");
    void printState(Print *stream);
    
  private:
    static Turntable *_firstTurntable;
    Turntable();
    static Turntable* create(uint16_t id, TurntableType turntableType,uint16_t homeAngle,VPIN extVpin, const char * description );
    
    // object member variables
    uint16_t id;
    TurntableType type;
    bool moving;
    uint8_t position;    
    byte previousPosition;
    char * description;
    VPIN extVpin; // for type ext
    uint16_t homeAngle; // Home position for the turntable     
    TurntablePosition * _firstPosition; // Linked list for positions
    Turntable *_nextTurntable; // Pointer to next turntable object
    
    // object functions
    void shareNodeToCS();
    bool setPositionInternal(uint8_t position, byte activity, bool tellNodes);
  
};

#endif
