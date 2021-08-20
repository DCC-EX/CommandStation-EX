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


//#define EESTOREDEBUG 
#include "defines.h"
#include "EEStore.h"
#include "StringFormatter.h"
#include "RMFT2.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

#include "DCC.h"
#include "LCN.h"

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

  virtual bool activate(bool close) = 0;  // Mandatory in subclass
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
  /* 
   * Virtual functions
   */
  virtual void print(Print *stream) {}
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

  static bool activate(uint16_t id, bool closeFlag);

  inline static bool setClosed(uint16_t id) {
    return activate(id, true);
  }

  inline static bool setThrown(uint16_t id) {
    return activate(id, false);
  }

  inline static bool setClosed(uint16_t id, bool close) {
    return activate(id, close);
  }

  static bool setClosedStateOnly(uint16_t id, bool close) {
    Turnout *tt = get(id);
    if (tt) return false;
    tt->_turnoutData.closed = close;
    return true;
  }

  inline static Turnout *first() { return _firstTurnout; }

  // Load all turnout definitions.
  static void load();
  // Load one turnout definition
  static Turnout *loadTurnout();
  // Save all turnout definitions
  static void store();

  static void printAll(Print *stream) {
    for (Turnout *tt = _firstTurnout; tt != 0; tt = tt->_nextTurnout)
      tt->print(stream);
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

public:
  // Constructor
  ServoTurnout(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed = true) :
    Turnout(id, TURNOUT_SERVO, closed) 
  {
    _servoTurnoutData.vpin = vpin;
    _servoTurnoutData.thrownPosition = thrownPosition; 
    _servoTurnoutData.closedPosition = closedPosition;
    _servoTurnoutData.profile = profile;
  }

  // Create function
  static Turnout *create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed = true) {
#ifndef IO_NO_HAL
    Turnout *tt = get(id);
    if (tt) { 
      // Object already exists, check if it is usable
      if (tt->isType(TURNOUT_SERVO)) {
        // Yes, so set parameters
        ServoTurnout *st = (ServoTurnout *)tt;
        st->_servoTurnoutData.vpin = vpin;
        st->_servoTurnoutData.thrownPosition = thrownPosition;
        st->_servoTurnoutData.closedPosition = closedPosition;
        st->_servoTurnoutData.profile = profile;
        // Don't touch the _closed parameter, retain the original value.

        // We don't really need to do the following, since a call to IODevice::_writeAnalogue 
        //  will provide all the data that is required!
        // int params[] = {(int)thrownPosition, (int)closedPosition, profile, closed};
        // IODevice::configure(vpin, IODevice::CONFIGURE_SERVO, 4, params);

        // Set position to saved position
        IODevice::writeAnalogue(vpin, closed ? closedPosition : thrownPosition, PCA9685::Instant);

        return tt;
      } else {
        // Incompatible object, delete and recreate
        remove(id);
      }
    }
    tt = (Turnout *)new ServoTurnout(id, vpin, thrownPosition, closedPosition, profile, closed);
    IODevice::writeAnalogue(vpin, closed ? closedPosition : thrownPosition, PCA9685::Instant);
    return tt;
#else
    return NULL;
#endif
  }

  // ServoTurnout-specific code for throwing or closing a servo turnout.
  bool activate(bool close) override {
#ifndef IO_NO_HAL
    IODevice::writeAnalogue(_servoTurnoutData.vpin, 
      close ? _servoTurnoutData.closedPosition : _servoTurnoutData.thrownPosition, _servoTurnoutData.profile);
    _turnoutData.closed = close;
#endif
    return true;
  }

  void save() override {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _servoTurnoutData);
    EEStore::advance(sizeof(_servoTurnoutData));
  }

  void print(Print *stream) override {
    StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), _turnoutData.id, _servoTurnoutData.vpin, 
      _servoTurnoutData.thrownPosition, _servoTurnoutData.closedPosition, _servoTurnoutData.profile, 
      _turnoutData.closed ^ useLegacyTurnoutBehaviour);
  }

  // Load a Servo turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData) {
    ServoTurnoutData servoTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), servoTurnoutData);
    EEStore::advance(sizeof(servoTurnoutData));
    
    // Create new object
    ServoTurnout *tt = new ServoTurnout(turnoutData->id, servoTurnoutData.vpin, servoTurnoutData.thrownPosition,
      servoTurnoutData.closedPosition, servoTurnoutData.profile, turnoutData->closed);

    return tt;
  }
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

public:
  // Constructor
  DCCTurnout(uint16_t id, uint16_t address, uint8_t subAdd) :
    Turnout(id, TURNOUT_DCC, false)
  {
    _dccTurnoutData.address = ((address-1) << 2) + subAdd + 1;
  }

  // Create function
  static Turnout *create(uint16_t id, uint16_t add, uint8_t subAdd) {
    Turnout *tt = get(id);
    if (tt) { 
      // Object already exists, check if it is usable
      if (tt->isType(TURNOUT_DCC)) {
        // Yes, so set parameters<T>
        DCCTurnout *dt = (DCCTurnout *)tt;
        dt->_dccTurnoutData.address = ((add-1) << 2) + subAdd + 1;
        // Don't touch the _closed parameter, retain the original value.
        return tt;
      } else {
        // Incompatible object, delete and recreate
        remove(id);
      }
    }
    tt = (Turnout *)new DCCTurnout(id, add, subAdd);
    return tt;
  }

  bool activate(bool close) override {
    DCC::setAccessory((((_dccTurnoutData.address-1) >> 2) + 1), 
      ((_dccTurnoutData.address-1) & 3), close);
    _turnoutData.closed = close;
    return true;
  }

  void save() override {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _dccTurnoutData);
    EEStore::advance(sizeof(_dccTurnoutData));
  }

  void print(Print *stream) override {
    StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), _turnoutData.id, 
      (((_dccTurnoutData.address-1) >> 2)+1), ((_dccTurnoutData.address-1) & 3), 
      _turnoutData.closed ^ useLegacyTurnoutBehaviour); 
  }

  // Load a DCC turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData) {
    DCCTurnoutData dccTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), dccTurnoutData);
    EEStore::advance(sizeof(dccTurnoutData));
    
    // Create new object
    DCCTurnout *tt = new DCCTurnout(turnoutData->id, (((dccTurnoutData.address-1) >> 2)+1), ((dccTurnoutData.address-1) & 3));

    return tt;
  }
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

public:
  // Constructor
  VpinTurnout(uint16_t id, VPIN vpin, bool closed=true) :
    Turnout(id, TURNOUT_VPIN, closed)
  {
    _vpinTurnoutData.vpin = vpin;
  }

  // Create function
  static Turnout *create(uint16_t id, VPIN vpin, bool closed=true) {
    Turnout *tt = get(id);
    if (tt) { 
      // Object already exists, check if it is usable
      if (tt->isType(TURNOUT_VPIN)) {
        // Yes, so set parameters
        VpinTurnout *vt = (VpinTurnout *)tt;
        vt->_vpinTurnoutData.vpin = vpin;
        // Don't touch the _closed parameter, retain the original value.
        return tt;
      } else {
        // Incompatible object, delete and recreate
        remove(id);
      }
    }
    tt = (Turnout *)new VpinTurnout(id, vpin, closed);
    return tt;
  }

  bool activate(bool close) override {
    IODevice::write(_vpinTurnoutData.vpin, close);
    _turnoutData.closed = close;
    return true;
  }

  void save() override {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _vpinTurnoutData);
    EEStore::advance(sizeof(_vpinTurnoutData));
  }

  void print(Print *stream) override {
    StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), _turnoutData.id, _vpinTurnoutData.vpin, 
      _turnoutData.closed ^ useLegacyTurnoutBehaviour); 
  }

  // Load a VPIN turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData) {
    VpinTurnoutData vpinTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), vpinTurnoutData);
    EEStore::advance(sizeof(vpinTurnoutData));
    
    // Create new object
    VpinTurnout *tt = new VpinTurnout(turnoutData->id, vpinTurnoutData.vpin, turnoutData->closed);

    return tt;
  }
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

public:
  // Constructor
  LCNTurnout(uint16_t id, bool closed=true) :
    Turnout(id, TURNOUT_LCN, closed)
  { }

  // Create function
  static Turnout *create(uint16_t id, bool closed=true) {
    Turnout *tt = get(id);
    if (tt) { 
      // Object already exists, check if it is usable
      if (tt->isType(TURNOUT_LCN)) {
        // Yes, so return this object
        return tt;
      } else {
        // Incompatible object, delete and recreate
        remove(id);
      }
    }
    tt = (Turnout *)new LCNTurnout(id, closed);
    return tt;
  }

  bool activate(bool close) override {
    // Assume that the LCN command still uses 1 for throw and 0 for close...
    LCN::send('T', _turnoutData.id, !close);
    // The _turnoutData.closed flag should be updated by a message from the LCN master, later.
    return true;
  }

  // LCN turnouts not saved to EEPROM.
  //void save() override {  }
  //static Turnout *load(struct TurnoutData *turnoutData) {

  void print(Print *stream) override {
    StringFormatter::send(stream, F("<H %d LCN %d>\n"), _turnoutData.id, 
    _turnoutData.closed ^ useLegacyTurnoutBehaviour); 
  }

};

