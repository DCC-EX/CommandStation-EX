/*
 *  © 2021 Restructured Neil McKechnie
 *  © 2013-2016 Gregg E. Berman
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

#ifndef TURNOUTS_H
#define TURNOUTS_H

//#define EESTOREDEBUG 
#include "Arduino.h"
#include "IODevice.h"


// Turnout type definitions
enum {
  TURNOUT_DCC = 1,
  TURNOUT_SERVO = 2,
  TURNOUT_VPIN = 3,
  TURNOUT_LCN = 4,
};

/*************************************************************************************
 * Turnout - Base class for turnouts.
 * 
 *************************************************************************************/

class Turnout {
protected:
  /* 
   * Object data
   */

  // The TurnoutData struct contains data common to all turnout types, that 
  // is written to EEPROM when the turnout is saved.
  // The first byte of this struct contains the 'closed' flag which is
  // updated whenever the turnout changes from thrown to closed and
  // vice versa.  If the turnout has been saved, then this byte is rewritten
  // when changed in RAM.  The 'closed' flag must be located in the first byte.
  struct TurnoutData {
    bool closed : 1;
    bool _rfu: 2;
    uint8_t turnoutType : 5;
    uint16_t id;
  } _turnoutData;  // 3 bytes

  // Address in eeprom of first byte of the _turnoutData struct (containing the closed flag).
  // Set to zero if the object has not been saved in EEPROM, e.g. for newly created Turnouts, and 
  // for all LCN turnouts.
  uint16_t _eepromAddress = 0;

  // Pointer to next turnout on linked list.
  Turnout *_nextTurnout = 0;

  /*
   * Constructor
   */
  Turnout(uint16_t id, uint8_t turnoutType, bool closed) {
    _turnoutData.id = id;
    _turnoutData.turnoutType = turnoutType;
    _turnoutData.closed = closed;
    add(this);
  }

  /* 
   * Static data
   */ 

  static Turnout *_firstTurnout;
  static int _turnoutlistHash;

  /* 
   * Virtual functions
   */

  virtual bool setClosedInternal(bool close) = 0;  // Mandatory in subclass
  virtual void save() {}
  
  /*
   * Static functions
   */

  static Turnout *get(uint16_t id);

  static void add(Turnout *tt);
  
public:
  /* 
   * Static data
   */
  static int turnoutlistHash;
  static bool useLegacyTurnoutBehaviour;

  /*
   * Public base class functions
   */
  inline bool isClosed() { return _turnoutData.closed; };
  inline bool isThrown() { return !_turnoutData.closed; }
  inline bool isType(uint8_t type) { return _turnoutData.turnoutType == type; }
  inline uint16_t getId() { return _turnoutData.id; }
  inline Turnout *next() { return _nextTurnout; }
  void printState(Print *stream);
  /* 
   * Virtual functions
   */
  virtual void print(Print *stream) {
    (void)stream;  // avoid compiler warnings.
  }
  virtual ~Turnout() {}   // Destructor

  /*
   * Public static functions
   */
  inline static bool exists(uint16_t id) { return get(id) != 0; }

  static bool remove(uint16_t id);

  static bool isClosed(uint16_t id);

  inline static bool isThrown(uint16_t id) {
    return !isClosed(id);
  }

  static bool setClosed(uint16_t id, bool closeFlag);

  inline static bool setClosed(uint16_t id) {
    return setClosed(id, true);
  }

  inline static bool setThrown(uint16_t id) {
    return setClosed(id, false);
  }

  static bool setClosedStateOnly(uint16_t id, bool close);

  inline static Turnout *first() { return _firstTurnout; }

  // Load all turnout definitions.
  static void load();
  // Load one turnout definition
  static Turnout *loadTurnout();
  // Save all turnout definitions
  static void store();

  static void printAll(Print *stream) {
    for (Turnout *tt = _firstTurnout; tt != 0; tt = tt->_nextTurnout)
      tt->printState(stream);
  }

  static void printState(uint16_t id, Print *stream);
};


/*************************************************************************************
 * ServoTurnout - Turnout controlled by servo device.
 * 
 *************************************************************************************/
class ServoTurnout : public Turnout {
private:
  // ServoTurnoutData contains data specific to this subclass that is 
  // written to EEPROM when the turnout is saved.
  struct ServoTurnoutData {
    VPIN vpin;
    uint16_t closedPosition : 12;
    uint16_t thrownPosition : 12;
    uint8_t profile;
  } _servoTurnoutData; // 6 bytes

  // Constructor
  ServoTurnout(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed = true);

public:
  // Create function
  static Turnout *create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed = true);

  // Load a Servo turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData);
  void print(Print *stream) override;

protected:
  // ServoTurnout-specific code for throwing or closing a servo turnout.
  bool setClosedInternal(bool close) override;
  void save() override;

};

/*************************************************************************************
 * DCCTurnout - Turnout controlled by DCC Accessory Controller.
 * 
 *************************************************************************************/
class DCCTurnout : public Turnout {
private:
  // DCCTurnoutData contains data specific to this subclass that is 
  // written to EEPROM when the turnout is saved.
  struct DCCTurnoutData {
    // DCC address (Address in bits 15-2, subaddress in bits 1-0
    uint16_t address; // CS currently supports linear address 1-2048
      // That's DCC accessory address 1-512 and subaddress 0-3.
  } _dccTurnoutData; // 2 bytes

  // Constructor
  DCCTurnout(uint16_t id, uint16_t address, uint8_t subAdd);

public:
  // Create function
  static Turnout *create(uint16_t id, uint16_t add, uint8_t subAdd);
  // Load a VPIN turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData);
  void print(Print *stream) override;

protected:
  bool setClosedInternal(bool close) override;
  void save() override;

};


/*************************************************************************************
 * VpinTurnout - Turnout controlled through a HAL vpin.
 * 
 *************************************************************************************/
class VpinTurnout : public Turnout {
private:
  // VpinTurnoutData contains data specific to this subclass that is 
  // written to EEPROM when the turnout is saved.
  struct VpinTurnoutData {
    VPIN vpin;
  } _vpinTurnoutData; // 2 bytes

  // Constructor
  VpinTurnout(uint16_t id, VPIN vpin, bool closed=true);

public:
  // Create function
  static Turnout *create(uint16_t id, VPIN vpin, bool closed=true);

  // Load a VPIN turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData);
  void print(Print *stream) override;

protected:
  bool setClosedInternal(bool close) override;
  void save() override;

};


/*************************************************************************************
 * LCNTurnout - Turnout controlled by Loconet
 * 
 *************************************************************************************/
class LCNTurnout : public Turnout {
private:
  // LCNTurnout has no specific data, and in any case is not written to EEPROM!
  // struct LCNTurnoutData {
  // } _lcnTurnoutData; // 0 bytes

  // Constructor
  LCNTurnout(uint16_t id, bool closed=true);

public:
  // Create function
  static Turnout *create(uint16_t id, bool closed=true);


  bool setClosedInternal(bool close) override;

  // LCN turnouts not saved to EEPROM.
  //void save() override {  }
  //static Turnout *load(struct TurnoutData *turnoutData) {

  void print(Print *stream) override;

};

#endif
