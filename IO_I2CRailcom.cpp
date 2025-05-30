   /*
 *  © 2024, Henk Kruisbrink & Chris Harlow. All rights reserved.
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
 * 
 * Dec 2023, Added NXP SC16IS752 I2C Dual UART
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be
 * needed as the RX Fifo holds the reply 
 * 
 * Jan 2024, Issue with using both UARTs simultaniously, the secod uart seems to work  but the first transmit 
 * corrupt data. This need more analysis and experimenatation. 
 * Will push this driver to the dev branch with the uart fixed to 0 
 * Both SC16IS750 (single uart) and SC16IS752 (dual uart, but only uart 0 is enable)
 * 
 * myHall.cpp configuration syntax:
 * 
 * I2CRailcom::create(1st vPin, vPins, I2C address);
 * 
 * myAutomation configuration
 *  HAL(I2CRailcom, 1st vPin, vPins, I2C address)
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated (to prevent overlaps)
 * I2C Address  : I2C address of the serial controller, in 0x format
 */

#include "IODevice.h"
#include "IO_I2CRailcom.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "Railcom.h"


I2CRailcom::I2CRailcom(VPIN firstVpin, int nPins, I2CAddress i2cAddress){
    _firstVpin = firstVpin;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
   } 
  
void I2CRailcom::create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) 
    new I2CRailcom(firstVpin,nPins,i2cAddress); 
    }

  void I2CRailcom::_begin() {
    I2CManager.setClock(1000000); // TODO do we need this?
    I2CManager.begin();
    auto exists=I2CManager.exists(_I2CAddress);
    DIAG(F("I2CRailcom: %s RailcomCollector %S detected"), 
           _I2CAddress.toString(), exists?F(""):F(" NOT"));
    if (!exists) return;
  
    _deviceState=DEVSTATE_NORMAL;
    _display();
    }
  
  
  void I2CRailcom::_loop(unsigned long currentMicros) {
    (void)currentMicros; // not used, but needed to match IODevice interface
    // Read responses from device
    if (_deviceState!=DEVSTATE_NORMAL) return;
    
    // have we read this cutout already?
    // basically we only poll once per packet when railcom cutout is working
    auto cut=DCCWaveform::getRailcomCutoutCounter();
    if (cutoutCounter==cut) return; 
    cutoutCounter=cut; 
    Railcom::loop(); // in case a csv read has timed out 
    
    // Obtain data length from the collector
    byte inbuf[1];
    byte queryLength[]={'?'};
    auto state=I2CManager.read(_I2CAddress, inbuf, 1,queryLength,sizeof(queryLength)); 
    if (state) {
      DIAG(F("RC ? state=%d"),state);
      return;
    }
    auto length=inbuf[0];
    if (length==0) return;  // nothing to report 

    // Build a buffer and import the data from the collector
    byte inbuf2[length];
    byte queryData[]={'>'};
    state=I2CManager.read(_I2CAddress, inbuf2, length,queryData,sizeof(queryData)); 
    if (state) {
      DIAG(F("RC > %d state=%d"),length,state);
      return;
    }
    
    // process incoming data buffer 
    Railcom::process(_firstVpin,inbuf2,length);
     
  }
          
 
  void I2CRailcom::_display() {
    DIAG(F("I2CRailcom: %s blocks %d-%d  %S"), _I2CAddress.toString(), _firstVpin, _firstVpin+_nPins-1,
      (_deviceState!=DEVSTATE_NORMAL) ? F("OFFLINE") : F(""));
  }
  
  