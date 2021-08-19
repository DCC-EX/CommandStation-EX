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


#include "defines.h"
#include "EEStore.h"
#include "StringFormatter.h"
#include "RMFT2.h"
#include "Turnouts.h"
#ifdef EESTOREDEBUG
#include "DIAG.h"
#endif

// Keywords used for turnout configuration.
const int16_t HASH_KEYWORD_SERVO=27709;
const int16_t HASH_KEYWORD_DCC=6436;
const int16_t HASH_KEYWORD_VPIN=-415;


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

  // Static activate function is invoked from close(), throw() etc. to perform the 
  //  common parts of the turnout operation.  Code which is specific to a turnout
  //  type should be placed in the polymorphic virtual function activate(bool) which is
  //  called from here.
  bool Turnout::activate(uint16_t id, bool closeFlag) { 
  #ifdef EESTOREDEBUG
    if (closeFlag) 
      DIAG(F("Turnout::close(%d)"), id);
    else
      DIAG(F("Turnout::throw(%d)"), id);
  #endif
    Turnout *tt = Turnout::get(id);
    if (!tt) return false;
    bool ok = tt->activate(closeFlag);

    // Write new closed/thrown state to EEPROM if required.  Note that eepromAddress
    // is always zero for LCN turnouts.
    if (EEStore::eeStore->data.nTurnouts > 0 && tt->_eepromAddress) 
      EEPROM.put(tt->_eepromAddress, tt->_turnoutData.closed);  

  #if defined(RMFT_ACTIVE)
    // TODO: Check that the inversion is correct here!
    RMFT2::turnoutEvent(id, !closeFlag);
  #endif  

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
    Turnout *tt;
    // Read turnout type from EEPROM
    struct TurnoutData turnoutData;
    int eepromAddress = EEStore::pointer(); // Address of byte containing the _closed flag.
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
    }
    if (!tt) {
      // Save EEPROM address in object.  Note that LCN turnouts always have eepromAddress of zero.
      tt->_eepromAddress = eepromAddress;
      add(tt);
    }

#ifdef EESTOREDEBUG
    printAll(&Serial);
#endif
    return tt;
  }

