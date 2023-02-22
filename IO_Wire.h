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
 * The purpose of this module is to provide an interface to the DCC
 * I2CManager that is compatible with code written for the Arduino 
 * 'Wire' interface.
 * 
 * To use it, just replace 
 *  #include "Wire.h" or #include <Wire.h>
 * with
 *  #include "IO_Wire.h"
 * 
 * Note that the CS only supports I2C master mode, so the calls related to
 * slave mode are not implemented here.
 * 
 */

#ifndef IO_WIRE
#define IO_WIRE

#include "IODevice.h"

#ifndef I2C_USE_WIRE

class IO_Wire : public IODevice, public Stream {
public:
  IO_Wire() {
    addDevice(this);
  };
  void begin() {
    I2CManager.begin();
  }
  void setClock(uint32_t speed) {
    I2CManager.setClock(speed);
  }
  void beginTransmission(uint8_t address) {
    i2cAddress = address;
    outputLength = 0;
  }
  size_t write(byte value) override {
    if (outputLength < sizeof(outputBuffer)) {
      outputBuffer[outputLength++] = value;
      return 1;
    } else
      return 0;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    for (size_t i=0; i<size; i++) {
      if (!write(buffer[i])) return i;
    }
    return size;
  }
  uint8_t endTransmission(bool) {
    // As this software doesn't run in a multi-master environment, there
    // is no advantage to holding the bus between transactions.  Therefore,
    // for simplicity, a stop condition is always sent.
    return I2CManager.write(i2cAddress, outputBuffer, outputLength);
  }
  uint8_t requestFrom(uint8_t address, uint8_t readSize, uint8_t sendStop) {
    (void)sendStop; // suppress compiler warning
    uint8_t status = I2CManager.read(address, inputBuffer, readSize);
    inputPos = 0;
    inputLength = readSize;
    return status;    
  }
  uint8_t requestFrom(uint8_t address, uint8_t quantity)
  {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
  }
  uint8_t requestFrom(int address, int quantity)
  {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
  }
  uint8_t requestFrom(int address, int quantity, int sendStop)
  {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
  }
  int read() override {
    if (inputPos < inputLength)
      return inputBuffer[inputPos++];
    else
      return -1;
  }
  int available() override {
    return (inputPos < inputLength);
  }
  int peek() override {
    if (inputPos < inputLength) 
      return inputBuffer[inputPos];
    else
      return -1;
  }
  uint8_t endTransmission() {
    return endTransmission(true);
  }

  static IO_Wire Wire();

protected:
  void _begin() { }
  void _display() {
    DIAG(F("I2CManager Wire Interface"));
  }

private:
  uint8_t outputBuffer[32];
  uint8_t outputLength = 0;
  uint8_t inputBuffer[32];
  uint8_t inputLength = 0;
  uint8_t inputPos = 0;
  uint8_t i2cAddress;
};

static IO_Wire Wire;

#else
#include <Wire.h>
#endif
#endif 