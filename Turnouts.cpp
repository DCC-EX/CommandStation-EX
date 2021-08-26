/*
 *  © 2021 Restructured Neil McKechnie
 *  © 2013-2016 Gregg E. Berman
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
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


#include "defines.h"  // includes config.h
#include "EEStore.h"
#include "StringFormatter.h"
#include "RMFT2.h"
#include "Turnouts.h"
#include "DCC.h"
#include "LCN.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

  /* 
   * Protected static data
   */ 

  Turnout *Turnout::_firstTurnout = 0;

  /* 
   * Public static data
   */
  int Turnout::turnoutlistHash = 0;
 
  /*
   * Protected static functions
   */

  Turnout *Turnout::get(uint16_t id) {
    // Find turnout object from list.
    for (Turnout *tt = _firstTurnout; tt != NULL; tt = tt->_nextTurnout)
      if (tt->_turnoutData.id == id) return tt;
    return NULL;
  }

  // Add new turnout to end of chain
  void Turnout::add(Turnout *tt) {
    if (!_firstTurnout) 
      _firstTurnout = tt;
    else {
      // Find last object on chain
      Turnout *ptr = _firstTurnout;
      for ( ; ptr->_nextTurnout!=0; ptr=ptr->_nextTurnout) {}
      // Line new object to last object.
      ptr->_nextTurnout = tt;
    }
    turnoutlistHash++;
  }
  
  // For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed; 
  void Turnout::printState(Print *stream) { 
    StringFormatter::send(stream, F("<H %d %d>\n"), 
      _turnoutData.id, !_turnoutData.closed);
  }

  // Remove nominated turnout from turnout linked list and delete the object.
  bool Turnout::remove(uint16_t id) {
    Turnout *tt,*pp=NULL;

    for(tt=_firstTurnout; tt!=NULL && tt->_turnoutData.id!=id; pp=tt, tt=tt->_nextTurnout) {}
    if (tt == NULL) return false;
    
    if (tt == _firstTurnout)
      _firstTurnout = tt->_nextTurnout;
    else
      pp->_nextTurnout = tt->_nextTurnout;

    delete (ServoTurnout *)tt;

    turnoutlistHash++;
    return true; 
  } 


  /*
   * Public static functions
   */

  bool Turnout::isClosed(uint16_t id) {
    Turnout *tt = get(id);
    if (tt) 
      return tt->isClosed();
    else
      return false;
  }

  bool Turnout::setClosedStateOnly(uint16_t id, bool close) {
    Turnout *tt = get(id);
    if (!tt) return false;
    tt->_turnoutData.closed = close;
    return true;
  }


  // Static setClosed function is invoked from close(), throw() etc. to perform the 
  //  common parts of the turnout operation.  Code which is specific to a turnout
  //  type should be placed in the virtual function setClosedInternal(bool) which is
  //  called from here.
  bool Turnout::setClosed(uint16_t id, bool closeFlag) { 
  #ifdef EESTOREDEBUG
    if (closeFlag) 
      DIAG(F("Turnout::close(%d)"), id);
    else
      DIAG(F("Turnout::throw(%d)"), id);
  #endif
    Turnout *tt = Turnout::get(id);
    if (!tt) return false;
    bool ok = tt->setClosedInternal(closeFlag);

    if (ok) {
      // Write byte containing new closed/thrown state to EEPROM if required.  Note that eepromAddress
      // is always zero for LCN turnouts.
      if (EEStore::eeStore->data.nTurnouts > 0 && tt->_eepromAddress > 0) 
        EEPROM.put(tt->_eepromAddress, tt->_turnoutData.flags);  

    #if defined(RMFT_ACTIVE)
      RMFT2::turnoutEvent(id, closeFlag);
    #endif

      // Send message to JMRI etc. over Serial USB.  This is done here
      // to ensure that the message is sent when the turnout operation
      // is not initiated by a Serial command.
      printState(id, &Serial);
    }
    return ok;
  }

  // Load all turnout objects
  void Turnout::load() {
    for (uint16_t i=0; i<EEStore::eeStore->data.nTurnouts; i++) {
      Turnout::loadTurnout();
    }
  }

  // Save all turnout objects
  void Turnout::store() {
    EEStore::eeStore->data.nTurnouts=0;
    for (Turnout *tt = _firstTurnout; tt != 0; tt = tt->_nextTurnout) {
      tt->save();
      EEStore::eeStore->data.nTurnouts++;
    }
  }

  // Load one turnout from EEPROM
  Turnout *Turnout::loadTurnout () {
    Turnout *tt = 0;
    // Read turnout type from EEPROM
    struct TurnoutData turnoutData;
    int eepromAddress = EEStore::pointer() + offsetof(struct TurnoutData, flags); // Address of byte containing the closed flag.
    EEPROM.get(EEStore::pointer(), turnoutData);
    EEStore::advance(sizeof(turnoutData));

    switch (turnoutData.turnoutType) {
      case TURNOUT_SERVO:
        // Servo turnout
        tt = ServoTurnout::load(&turnoutData);
        break;
      case TURNOUT_DCC:
        // DCC Accessory turnout
        tt = DCCTurnout::load(&turnoutData);
        break;
      case TURNOUT_VPIN:
        // VPIN turnout
        tt = VpinTurnout::load(&turnoutData);
        break;
      default:
        // If we find anything else, then we don't know what it is or how long it is, 
        // so we can't go any further through the EEPROM!
        return NULL;
    }
    if (tt) {
      // Save EEPROM address in object.  Note that LCN turnouts always have eepromAddress of zero.
      tt->_eepromAddress = eepromAddress + offsetof(struct TurnoutData, flags);
    }

#ifdef EESTOREDEBUG
    printAll(&Serial);
#endif
    return tt;
  }

  // Display, on the specified stream, the current state of the turnout (1=thrown or 0=closed).
  void Turnout::printState(uint16_t id, Print *stream) {
    Turnout *tt = get(id);
    if (!tt) tt->printState(stream);
  }


/*************************************************************************************
 * ServoTurnout - Turnout controlled by servo device.
 * 
 *************************************************************************************/

  // Private Constructor
  ServoTurnout::ServoTurnout(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed) :
    Turnout(id, TURNOUT_SERVO, closed) 
  {
    _servoTurnoutData.vpin = vpin;
    _servoTurnoutData.thrownPosition = thrownPosition; 
    _servoTurnoutData.closedPosition = closedPosition;
    _servoTurnoutData.profile = profile;
  }

  // Create function
  Turnout *ServoTurnout::create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed) {
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

        // Set position directly to specified position - we don't know where it is moving from.
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
    (void)id; (void)vpin; (void)thrownPosition; (void)closedPosition;
    (void)profile; (void)closed;          // avoid compiler warnings.
    return NULL;
#endif
  }

  // Load a Servo turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  Turnout *ServoTurnout::load(struct TurnoutData *turnoutData) {
    ServoTurnoutData servoTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), servoTurnoutData);
    EEStore::advance(sizeof(servoTurnoutData));
    
    // Create new object
    Turnout *tt = ServoTurnout::create(turnoutData->id, servoTurnoutData.vpin, servoTurnoutData.thrownPosition,
      servoTurnoutData.closedPosition, servoTurnoutData.profile, turnoutData->closed);
    return tt;
  }

  // For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed
  void ServoTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), _turnoutData.id, _servoTurnoutData.vpin, 
      _servoTurnoutData.thrownPosition, _servoTurnoutData.closedPosition, _servoTurnoutData.profile, 
      !_turnoutData.closed);
  }

  // ServoTurnout-specific code for throwing or closing a servo turnout.
  bool ServoTurnout::setClosedInternal(bool close) {
#ifndef IO_NO_HAL
    IODevice::writeAnalogue(_servoTurnoutData.vpin, 
      close ? _servoTurnoutData.closedPosition : _servoTurnoutData.thrownPosition, _servoTurnoutData.profile);
    _turnoutData.closed = close;
#else
    (void)close;  // avoid compiler warnings
#endif
    return true;
  }

  void ServoTurnout::save() {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _servoTurnoutData);
    EEStore::advance(sizeof(_servoTurnoutData));
  }

/*************************************************************************************
 * DCCTurnout - Turnout controlled by DCC Accessory Controller.
 * 
 *************************************************************************************/

#if defined(DCC_TURNOUTS_RCN_213)
  const bool DCCTurnout::rcn213Compliant = true;
#else
  const bool DCCTurnout::rcn213Compliant = false;
#endif

  // DCCTurnoutData contains data specific to this subclass that is 
  // written to EEPROM when the turnout is saved.
  struct DCCTurnoutData {
    // DCC address (Address in bits 15-2, subaddress in bits 1-0
    uint16_t address; // CS currently supports linear address 1-2048
      // That's DCC accessory address 1-512 and subaddress 0-3.
  } _dccTurnoutData; // 2 bytes

  // Constructor
  DCCTurnout::DCCTurnout(uint16_t id, uint16_t address, uint8_t subAdd) :
    Turnout(id, TURNOUT_DCC, false)
  {
    _dccTurnoutData.address = ((address-1) << 2) + subAdd + 1;
  }

  // Create function
  Turnout *DCCTurnout::create(uint16_t id, uint16_t add, uint8_t subAdd) {
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

  // Load a DCC turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  Turnout *DCCTurnout::load(struct TurnoutData *turnoutData) {
    DCCTurnoutData dccTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), dccTurnoutData);
    EEStore::advance(sizeof(dccTurnoutData));
    
    // Create new object
    DCCTurnout *tt = new DCCTurnout(turnoutData->id, (((dccTurnoutData.address-1) >> 2)+1), ((dccTurnoutData.address-1) & 3));

    return tt;
  }

  void DCCTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), _turnoutData.id, 
      (((_dccTurnoutData.address-1) >> 2)+1), ((_dccTurnoutData.address-1) & 3), 
      !_turnoutData.closed); 
    // Also report using classic DCC++ syntax for DCC accessory turnouts, since JMRI expects this.
    StringFormatter::send(stream, F("<H %d %d %d %d>\n"), _turnoutData.id, 
      (((_dccTurnoutData.address-1) >> 2)+1), ((_dccTurnoutData.address-1) & 3), 
      !_turnoutData.closed); 
  }

  bool DCCTurnout::setClosedInternal(bool close) {
    // DCC++ Classic behaviour is that Throw writes a 1 in the packet,
    // and Close writes a 0.  
    // RCN-213 specifies that Throw is 0 and Close is 1.
    DCC::setAccessory((((_dccTurnoutData.address-1) >> 2) + 1), 
      ((_dccTurnoutData.address-1) & 3), close ^ !rcn213Compliant);
    _turnoutData.closed = close;
    return true;
  }

  void DCCTurnout::save() {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _dccTurnoutData);
    EEStore::advance(sizeof(_dccTurnoutData));
  }



/*************************************************************************************
 * VpinTurnout - Turnout controlled through a HAL vpin.
 * 
 *************************************************************************************/

  // Constructor
  VpinTurnout::VpinTurnout(uint16_t id, VPIN vpin, bool closed) :
    Turnout(id, TURNOUT_VPIN, closed)
  {
    _vpinTurnoutData.vpin = vpin;
  }

  // Create function
  Turnout *VpinTurnout::create(uint16_t id, VPIN vpin, bool closed) {
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

  // Load a VPIN turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  Turnout *VpinTurnout::load(struct TurnoutData *turnoutData) {
    VpinTurnoutData vpinTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), vpinTurnoutData);
    EEStore::advance(sizeof(vpinTurnoutData));
    
    // Create new object
    VpinTurnout *tt = new VpinTurnout(turnoutData->id, vpinTurnoutData.vpin, turnoutData->closed);

    return tt;
  }

  // Report 1 for thrown, 0 for closed.
  void VpinTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), _turnoutData.id, _vpinTurnoutData.vpin, 
      !_turnoutData.closed); 
  }

  bool VpinTurnout::setClosedInternal(bool close) {
    IODevice::write(_vpinTurnoutData.vpin, close);
    _turnoutData.closed = close;
    return true;
  }

  void VpinTurnout::save() {
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _vpinTurnoutData);
    EEStore::advance(sizeof(_vpinTurnoutData));
  }


/*************************************************************************************
 * LCNTurnout - Turnout controlled by Loconet
 * 
 *************************************************************************************/

  // LCNTurnout has no specific data, and in any case is not written to EEPROM!
  // struct LCNTurnoutData {
  // } _lcnTurnoutData; // 0 bytes

  // Constructor
  LCNTurnout::LCNTurnout(uint16_t id, bool closed) :
    Turnout(id, TURNOUT_LCN, closed)
  { }

  // Create function
  Turnout *LCNTurnout::create(uint16_t id, bool closed) {
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

  bool LCNTurnout::setClosedInternal(bool close) {
    // Assume that the LCN command still uses 1 for throw and 0 for close...
    LCN::send('T', _turnoutData.id, !close);
    // The _turnoutData.closed flag should be updated by a message from the LCN master, later.
    return true;
  }

  // LCN turnouts not saved to EEPROM.
  //void save() override {  }
  //static Turnout *load(struct TurnoutData *turnoutData) {

  // Report 1 for thrown, 0 for closed.
  void LCNTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d LCN %d>\n"), _turnoutData.id, 
    !_turnoutData.closed); 
  }

