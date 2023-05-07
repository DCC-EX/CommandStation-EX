/*
 *  © 2021 Neil McKechnie
 *  © 2021 M Steve Todd
 *  © 2021 Fred Decker
 *  © 2020-2021 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2013-2016 Gregg E. Berman
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


#include "defines.h"  // includes config.h
#ifndef DISABLE_EEPROM
#include "EEStore.h"
#endif
#include "StringFormatter.h"
#include "CommandDistributor.h"
#include "EXRAIL2.h"
#include "Turnouts.h"
#include "DCC.h"
#include "LCN.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

  /* 
   * Protected static data
   */ 

  /* static */ Turnout *Turnout::_firstTurnout = 0;

  /* 
   * Public static data
   */
  /* static */ int Turnout::turnoutlistHash = 0;
 
  /*
   * Protected static functions
   */

  /* static */ Turnout *Turnout::get(uint16_t id) {
    // Find turnout object from list.
    for (Turnout *tt = _firstTurnout; tt != NULL; tt = tt->_nextTurnout)
      if (tt->_turnoutData.id == id) return tt;
    return NULL;
  }

  // Add new turnout to end of chain
  /* static */ void Turnout::add(Turnout *tt) {
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
  
  

  // Remove nominated turnout from turnout linked list and delete the object.
  /* static */ bool Turnout::remove(uint16_t id) {
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

  /* static */ bool Turnout::isClosed(uint16_t id) {
    Turnout *tt = get(id);
    if (tt) 
      return tt->isClosed();
    else
      return false;
  }

  /* static */ bool Turnout::setClosedStateOnly(uint16_t id, bool closeFlag) {
    Turnout *tt = get(id);
    if (!tt) return false;
    // I know it says setClosedStateOnly, but we need to tell others
    // that the state has changed too. But we only broadcast if there
    // really has been a change.
    if (tt->_turnoutData.closed != closeFlag) {
      tt->_turnoutData.closed = closeFlag;
      CommandDistributor::broadcastTurnout(id, closeFlag);
    }
#if defined(EXRAIL_ACTIVE)
    RMFT2::turnoutEvent(id, closeFlag);
#endif
    return true;
  }

#define DIAG_IO
  // Static setClosed function is invoked from close(), throw() etc. to perform the 
  //  common parts of the turnout operation.  Code which is specific to a turnout
  //  type should be placed in the virtual function setClosedInternal(bool) which is
  //  called from here.
  /* static */ bool Turnout::setClosed(uint16_t id, bool closeFlag) { 
#if defined(DIAG_IO)
    DIAG(F("Turnout(%d,%c)"), id, closeFlag ? 'c':'t');
#endif
    Turnout *tt = Turnout::get(id);
    if (!tt) return false;
    bool ok = tt->setClosedInternal(closeFlag);

    if (ok) {
      tt->setClosedStateOnly(id, closeFlag);
#ifndef DISABLE_EEPROM
      // Write byte containing new closed/thrown state to EEPROM if required.  Note that eepromAddress
      // is always zero for LCN turnouts.
      if (EEStore::eeStore->data.nTurnouts > 0 && tt->_eepromAddress > 0) 
        EEPROM.put(tt->_eepromAddress, tt->_turnoutData.flags);
#endif
    }
    return ok;
  }

#ifndef DISABLE_EEPROM
  // Load all turnout objects
  /* static */ void Turnout::load() {
    for (uint16_t i=0; i<EEStore::eeStore->data.nTurnouts; i++) {
      Turnout::loadTurnout();
    }
  }

  // Save all turnout objects
  /* static */ void Turnout::store() {
    EEStore::eeStore->data.nTurnouts=0;
    for (Turnout *tt = _firstTurnout; tt != 0; tt = tt->_nextTurnout) {
      tt->save();
      EEStore::eeStore->data.nTurnouts++;
    }
  }

  // Load one turnout from EEPROM
  /* static */ Turnout *Turnout::loadTurnout () {
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
    printAll(&USB_SERIAL);
#endif
    return tt;
  }
#endif

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
  /* static */ Turnout *ServoTurnout::create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed) {
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
        //  will provide all the data that is required!  However, if someone has configured 
        //  a Turnout, we should ensure that the SET() RESET() and other commands that use write() 
        //  behave consistently with the turnout commands.
        IODevice::configureServo(vpin, thrownPosition, closedPosition, profile, 0, closed);

        // Set position directly to specified position - we don't know where it is moving from.
        IODevice::writeAnalogue(vpin, closed ? closedPosition : thrownPosition, PCA9685::Instant);

        return tt;
      } else {
        // Incompatible object, delete and recreate
        remove(id);
      }
    }
    tt = (Turnout *)new ServoTurnout(id, vpin, thrownPosition, closedPosition, profile, closed);
    DIAG(F("Turnout 0x%x size %d size %d"), tt, sizeof(Turnout),sizeof(struct TurnoutData));
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
#ifndef DISABLE_EEPROM
    ServoTurnoutData servoTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), servoTurnoutData);
    EEStore::advance(sizeof(servoTurnoutData));
    
    // Create new object
    Turnout *tt = ServoTurnout::create(turnoutData->id, servoTurnoutData.vpin, servoTurnoutData.thrownPosition,
      servoTurnoutData.closedPosition, servoTurnoutData.profile, turnoutData->closed);
    return tt;
#else
    (void)turnoutData;
    return NULL;
#endif
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
#else
    (void)close;  // avoid compiler warnings
#endif
    return true;
  }

  void ServoTurnout::save() {
#ifndef DISABLE_EEPROM
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _servoTurnoutData);
    EEStore::advance(sizeof(_servoTurnoutData));
#endif
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
    _dccTurnoutData.address = address;
    _dccTurnoutData.subAddress = subAdd;
  }

  // Create function
  /* static */ Turnout *DCCTurnout::create(uint16_t id, uint16_t add, uint8_t subAdd) {
    Turnout *tt = get(id);
    if (tt) { 
      // Object already exists, check if it is usable
      if (tt->isType(TURNOUT_DCC)) {
        // Yes, so set parameters<T>
        DCCTurnout *dt = (DCCTurnout *)tt;
        dt->_dccTurnoutData.address = add;
        dt->_dccTurnoutData.subAddress = subAdd;
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
  /* static */ Turnout *DCCTurnout::load(struct TurnoutData *turnoutData) {
#ifndef DISABLE_EEPROM
    DCCTurnoutData dccTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), dccTurnoutData);
    EEStore::advance(sizeof(dccTurnoutData));
    
    // Create new object
    DCCTurnout *tt = new DCCTurnout(turnoutData->id, dccTurnoutData.address, dccTurnoutData.subAddress);

    return tt;
#else
    (void)turnoutData;
    return NULL;
#endif
  }

  void DCCTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d DCC %d %d %d>\n"), _turnoutData.id, 
      _dccTurnoutData.address, _dccTurnoutData.subAddress, !_turnoutData.closed); 
    // Also report using classic DCC++ syntax for DCC accessory turnouts, since JMRI expects this.
    StringFormatter::send(stream, F("<H %d %d %d %d>\n"), _turnoutData.id, 
      _dccTurnoutData.address, _dccTurnoutData.subAddress, !_turnoutData.closed); 
  }

  bool DCCTurnout::setClosedInternal(bool close) {
    // DCC++ Classic behaviour is that Throw writes a 1 in the packet,
    // and Close writes a 0.  
    // RCN-213 specifies that Throw is 0 and Close is 1.
    DCC::setAccessory(_dccTurnoutData.address, _dccTurnoutData.subAddress, close ^ !rcn213Compliant);
    return true;
  }

  void DCCTurnout::save() {
#ifndef DISABLE_EEPROM
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _dccTurnoutData);
    EEStore::advance(sizeof(_dccTurnoutData));
#endif
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
  /* static */ Turnout *VpinTurnout::create(uint16_t id, VPIN vpin, bool closed) {
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
  /* static */ Turnout *VpinTurnout::load(struct TurnoutData *turnoutData) {
#ifndef DISABLE_EEPROM
    VpinTurnoutData vpinTurnoutData;
    // Read class-specific data from EEPROM
    EEPROM.get(EEStore::pointer(), vpinTurnoutData);
    EEStore::advance(sizeof(vpinTurnoutData));
    
    // Create new object
    VpinTurnout *tt = new VpinTurnout(turnoutData->id, vpinTurnoutData.vpin, turnoutData->closed);

    return tt;
#else
    (void)turnoutData;
    return NULL;
#endif
  }

  // Report 1 for thrown, 0 for closed.
  void VpinTurnout::print(Print *stream) {
    StringFormatter::send(stream, F("<H %d VPIN %d %d>\n"), _turnoutData.id, _vpinTurnoutData.vpin, 
      !_turnoutData.closed); 
  }

  bool VpinTurnout::setClosedInternal(bool close) {
    IODevice::write(_vpinTurnoutData.vpin, close);
    return true;
  }

  void VpinTurnout::save() {
#ifndef DISABLE_EEPROM
    // Write turnout definition and current position to EEPROM
    // First write common servo data, then
    // write the servo-specific data
    EEPROM.put(EEStore::pointer(), _turnoutData);
    EEStore::advance(sizeof(_turnoutData));
    EEPROM.put(EEStore::pointer(), _vpinTurnoutData);
    EEStore::advance(sizeof(_vpinTurnoutData));
#endif
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
  /* static */ Turnout *LCNTurnout::create(uint16_t id, bool closed) {
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
    // The _turnoutData.closed flag should be updated by a message from the LCN master.
    // but in this implementation it is updated in setClosedStateOnly() instead.
    // If the LCN master updates this, setClosedStateOnly() and all setClosedInternal()
    // have to be updated accordingly so that the closed flag is only set once.
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

