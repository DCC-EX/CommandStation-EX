/*
 *  © 2023 Peter Cole
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

// Turntable type definitions
// EXTT = EX-Turntable
// DCC = DCC accessory turntables - to be added later
enum {
  TURNTABLE_EXTT = 1,
  // TURNTABLE_DCC = 2,
};

/*************************************************************************************
 * Turntable - Base class for turntables.
 * 
 *************************************************************************************/

class Turntable {
protected:
  /*
   * Object data
   */

  //  Data common to all turntable types
  struct TurntableData {
    union {
      struct {
        bool hidden : 1;
        uint8_t turntableType : 2;
        uint8_t position : 5;       // Allows up to 38 positions including 0/home
      };
      uint8_t flags;
    };
    uint16_t id;
  } _turntableData;

  // Linked list to store either positions (EXTT) or DCC addresses (DCC)
  struct Position {
    int16_t position;
    Position *next;
  };

  // Pointer to next turntable object
  Turntable *_nextTurntable = 0;

  /*
   * Constructor
   */
  Turntable(uint16_t id, uint8_t turntableType) {
    _turntableData.id = id;
    _turntableData.turntableType = turntableType;
    _turntableData.hidden = false;
    add(this);
  }

  /*
   * Static data
   */
  static Turntable *_firstTurntable;
  static int _turntablelistHash;

  /*
   * Virtual functions
   */
  virtual bool setPositionInternal(uint8_t position, uint8_t activity) = 0;

  /*
   * Static functions
   */
  static void add(Turntable *tto);

public:
  static Turntable *get(uint16_t id);

  /*
   * Static data
   */
  static int turntablelistHash;

  /*
   * Public base class functions
   */
  inline uint8_t getPosition() { return _turntableData.position; }
  inline bool isHidden() { return _turntableData.hidden; }
  inline void setHidden(bool h) {_turntableData.hidden=h; }
  inline bool isType(uint8_t type) { return _turntableData.turntableType == type; }
  inline uint16_t getId() { return _turntableData.id; }
  inline Turntable *next() { return _nextTurntable; }
  void printState(Print *stream);
  

  /*
   * Virtual functions
   */
  virtual void print(Print *stream) {
    (void)stream; // suppress compiler warnings
  }
  virtual ~Turntable() {} // Destructor
  

  /*
   * Public static functions
   */
  inline static bool exists(uint16_t id) { return get(id) != 0; }
  static bool remove(uint16_t id);
  static bool isPosition(uint16_t id, uint8_t position);
  static bool setPosition(uint16_t id, uint8_t position, uint8_t activity=0);
  static bool setPositionStateOnly(uint16_t id, uint8_t position);
  inline static Turntable *first() { return _firstTurntable; }
  static bool printAll(Print *stream) {
    bool gotOne = false;
    for (Turntable *tto = _firstTurntable; tto != 0; tto = tto->_nextTurntable)
      if (!tto->isHidden()) {
        gotOne = true;
        StringFormatter::send(stream, F("<i %d %d>\n"), tto->getId(), tto->getPosition());
      }
    return gotOne;
  }

};

/*************************************************************************************
 * EXTTTurntable - EX-Turntable device.
 * 
 *************************************************************************************/
class EXTTTurntable : public Turntable {
private:
  // EXTTTurntableData contains device specific data
  struct EXTTTurntableData {
    uint8_t i2caddress;
    VPIN vpin;
  } _exttTurntableData;

  // Constructor
  EXTTTurntable(uint16_t id, uint8_t i2caddress, VPIN vpin);

public:
  // Create function
  static Turntable *create(uint16_t id, uint8_t i2caddress, VPIN vpin);
  void print(Print *stream) override;

protected:
  // EX-Turntable specific code for setting position
  bool setPositionInternal(uint8_t position, uint8_t activity) override;

};

#endif
