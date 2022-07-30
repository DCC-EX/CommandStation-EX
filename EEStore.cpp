/*
 *  © 2021 Neil McKechnie
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
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

#include "defines.h"
#ifndef DISABLE_EEPROM
#include "EEStore.h"

#include "DIAG.h"
#include "Outputs.h"
#include "Sensors.h"
#include "Turnouts.h"

#if defined(ARDUINO_ARCH_SAMC)
ExternalEEPROM EEPROM;
#endif

void EEStore::init() {
#if defined(ARDUINO_ARCH_SAMC)
  EEPROM.begin(0x50);  // Address for Microchip 24-series EEPROM with all three
                       // A pins grounded (0b1010000 = 0x50)
#endif

  eeStore = (EEStore *)calloc(1, sizeof(EEStore));

  EEPROM.get(0, eeStore->data);  // get eeStore data

  // check to see that eeStore contains valid DCC++ ID
  if (strncmp(eeStore->data.id, EESTORE_ID, sizeof(EESTORE_ID)) != 0) {  
    // if not, create blank eeStore structure (no
    // turnouts, no sensors) and save it back to EEPROM  
    strncpy(eeStore->data.id, EESTORE_ID, sizeof(EESTORE_ID)+0);  
    eeStore->data.nTurnouts = 0;
    eeStore->data.nSensors = 0;
    eeStore->data.nOutputs = 0;
    EEPROM.put(0, eeStore->data);
  }

  reset();          // set memory pointer to first free EEPROM space
  Turnout::load();  // load turnout definitions
  Sensor::load();   // load sensor definitions
  Output::load();   // load output definitions
}

///////////////////////////////////////////////////////////////////////////////

void EEStore::clear() {
  sprintf(eeStore->data.id,
          EESTORE_ID);  // create blank eeStore structure (no turnouts, no
                        // sensors) and save it back to EEPROM
  eeStore->data.nTurnouts = 0;
  eeStore->data.nSensors = 0;
  eeStore->data.nOutputs = 0;
  EEPROM.put(0, eeStore->data);
}

///////////////////////////////////////////////////////////////////////////////

void EEStore::store() {
  reset();
  Turnout::store();
  Sensor::store();
  Output::store();
  EEPROM.put(0, eeStore->data);
  DIAG(F("EEPROM used: %d/%d bytes"), EEStore::pointer(), EEPROM.length());
}

///////////////////////////////////////////////////////////////////////////////

void EEStore::advance(int n) { eeAddress += n; }

///////////////////////////////////////////////////////////////////////////////

void EEStore::reset() { eeAddress = sizeof(EEStore); }
///////////////////////////////////////////////////////////////////////////////

int EEStore::pointer() { return (eeAddress); }
///////////////////////////////////////////////////////////////////////////////

void EEStore::dump(int num) {
  byte b = 0;
  DIAG(F("Addr  0x  char"));
  for (int n = 0; n < num; n++) {
    EEPROM.get(n, b);
    DIAG(F("%d     %x    %c"), n, b, isprint(b) ? b : ' ');
  }
}
///////////////////////////////////////////////////////////////////////////////

EEStore *EEStore::eeStore = NULL;
int EEStore::eeAddress = 0;
#endif
