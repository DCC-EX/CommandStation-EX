/*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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
 * This device driver monitors the state of turnout objects and writes updates,
 * on change of state, to an external 24C128 (16kByte) or 24C256 (32kByte)
 * EEPROM device connected via I2C.
 * 
 * When the device is restarted, it repositions the turnouts in accordance
 * with the last saved position.
 * 
 * To create a device instance, 
 *    IO_ExternalEEPROM::create(0, 0, i2cAddress);
 * 
 * 
 */

#ifndef IO_EXTERNALEEPROM_H
#define IO_EXTERNALEEPROM_H

#include "IODevice.h"
#include "I2CManager.h"
#include "Turnouts.h"

class ExternalEEPROM : public IODevice {
private:
  // Here we define the device-specific variables.  
  int _sizeInKBytes = 128;
  Turnout *_turnout = 0;
  int _lastTurnoutHash = 0;
  I2CRB _rb;
  uint8_t _buffer[32]; // 32 is max for Wire write

public:
  //  Static function to handle "IO_ExampleSerial::create(...)" calls.
  static void create(I2CAddress i2cAddress, int sizeInKBytes) {
    if (checkNoOverlap(0, 0, i2cAddress)) new ExternalEEPROM(i2cAddress, sizeInKBytes);
  } 

protected:
  // Constructor. 
  ExternalEEPROM(I2CAddress i2cAddress, int sizeInKBytes) {
    _I2CAddress = i2cAddress;
    _sizeInKBytes = sizeInKBytes;

    // Set up I2C structures.
    _rb.setWriteParams(_I2CAddress, _buffer, 32);

    addDevice(this);
  }

  // Device-specific initialisation
  void _begin() override {
    I2CManager.begin();
    I2CManager.setClock(1000000); // Max supported speed

    if (I2CManager.exists(_I2CAddress)) {
      // Initialise or read contents of EEPROM
      // and set turnout states accordingly.
      // Read 32 bytes from address 0x0000.
      I2CManager.read(_I2CAddress, _buffer, 32, 2, 0, 0);
      // Dump data
      DIAG(F("EEPROM First 32 bytes:"));
      for (int i=0; i<32; i+=8) 
        DIAG(F("%d: %x %x %x %x %x %x %x %x"), 
          i, _buffer[i], _buffer[i+1], _buffer[i+2], _buffer[i+3],
          _buffer[i+4], _buffer[i+5], _buffer[i+6], _buffer[i+7]);

#if defined(DIAG_IO)
      _display();
#endif
    } else {
      DIAG(F("ExternalEEPROM not found, I2C:%s"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
    }
  }
  
  // Loop function to do background scanning of the turnouts
  void _loop(unsigned long currentMicros) {
    (void)currentMicros;  // Suppress compiler warnings

    if (_rb.isBusy()) return;  // Can't do anything until previous request has completed.
    if (_rb.status == I2C_STATUS_NEGATIVE_ACKNOWLEDGE) {
      // Device not responding, probably still writing data, so requeue request
      I2CManager.queueRequest(&_rb);
      return;
    }

    if (_lastTurnoutHash != Turnout::turnoutlistHash) {
      _lastTurnoutHash = Turnout::turnoutlistHash;
      // Turnout list has changed, so pointer held from last run may be invalid
      _turnout = 0;  // Start at the beginning of the list again.
//#if defined(DIAG_IO)
      DIAG(F("Turnout Hash Changed!"));
//#endif
    }

    // Locate next turnout, or first one if there is no current one.
    if (_turnout) 
      _turnout = _turnout->next();
    else
      _turnout = Turnout::first();

    // Retrieve turnout state
    int turnoutID = _turnout->getId();
    int turnoutState = _turnout->isThrown();
    (void)turnoutID; // Suppress compiler warning
    (void)turnoutState; // Suppress compiler warning

    // TODO: Locate turnoutID in EEPROM (or EEPROM copy) and check if state has changed.
    // TODO: If it has, then initiate a write of the updated state to EEPROM

    delayUntil(currentMicros+5000);  // Write cycle time is 5ms max for FT24C256
  }

  // Display information about the device.
  void _display() {
    DIAG(F("ExternalEEPROM %dkBytes I2C:%s %S"), _sizeInKBytes, _I2CAddress.toString(),
      _deviceState== DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }


};

#endif // IO_EXTERNALEEPROM_H