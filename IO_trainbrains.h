/*
 *  © 2023, Chris Harlow. All rights reserved.
 *  © 2021, Neil McKechnie. All rights reserved.
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

#ifndef io_trainbrains_h
#define io_trainbrains_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for trainbrains 3-block occupancy detector.
 * For details see http://trainbrains.eu
 */
 
 enum TrackUnoccupancy
{
    TRACK_UNOCCUPANCY_UNKNOWN = 0,
    TRACK_OCCUPIED = 1,
    TRACK_UNOCCUPIED = 2
};

class Trainbrains02 : public GPIOBase<uint16_t> {
public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new Trainbrains02(vpin, nPins, i2cAddress);
  }

private:  
  // Constructor
  Trainbrains02(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) 
    : GPIOBase<uint16_t>((FSH *)F("Trainbrains02"), vpin, nPins, i2cAddress, interruptPin) 
  {
    requestBlock.setRequestParams(_I2CAddress, inputBuffer, sizeof(inputBuffer),
      outputBuffer, sizeof(outputBuffer));
    
     outputBuffer[0] = (uint8_t)_I2CAddress; // strips away the mux part.
     outputBuffer[1] =14;
     outputBuffer[2] =1;
     outputBuffer[3] =0;  // This is the channel updated at each poling call
     outputBuffer[4] =0;
     outputBuffer[5] =0;
     outputBuffer[6] =0;
     outputBuffer[7] =0;
     outputBuffer[8] =0;
     outputBuffer[9] =0;
  }
  
  void _writeGpioPort() override {}

  void _readGpioPort(bool immediate) override {
    // cycle channel on device each time 
    outputBuffer[3]=channelInProgress+1; // 1-origin 
    channelInProgress++;
    if(channelInProgress>=_nPins) channelInProgress=0; 
    
    if (immediate) {
      _processCompletion(I2CManager.read(_I2CAddress, inputBuffer, sizeof(inputBuffer), 
                                   outputBuffer, sizeof(outputBuffer)));
    } else {
      // Queue new request
      requestBlock.wait(); // Wait for preceding operation to complete
      // Issue new request to read GPIO register
      I2CManager.queueRequest(&requestBlock);
    }
  }

  // This function is invoked when an I/O operation on the requestBlock completes.
  void _processCompletion(uint8_t status) override {
    if (status != I2C_STATUS_OK) inputBuffer[6]=TRACK_UNOCCUPANCY_UNKNOWN;  
    if (inputBuffer[6] == TRACK_UNOCCUPIED ) _portInputState |=  0x01 <<channelInProgress;
    else _portInputState &=  ~(0x01 <<channelInProgress);
  }

  uint8_t channelInProgress=0; 
  uint8_t outputBuffer[10];
  uint8_t inputBuffer[10];
  
};

#endif