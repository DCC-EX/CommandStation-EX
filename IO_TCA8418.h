/*
 *  © 2023, Paul M. Antoine
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC-EX API
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

#ifndef io_tca8418_h
#define io_tca8418_h

#include "IO_GPIOBase.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for TCA8418 80-key keypad encoder, which we'll treat as 64 of the possible
 * 80 inputs for now, in an 8x8 matrix only, although the datasheet says:
 * 
 * The TCA8418 can be configured to support many different configurations of keypad setups.
 * All 18 GPIOs for the rows and columns can be used to support up to 80 keys in an 8x10 key pad
 * array. Another option is that all 18 GPIOs be used for GPIs to read 18 buttons which are
 * not connected in an array. Any combination in between is also acceptable (for example, a
 * 3x4 keypad matrix and using the remaining 11 GPIOs as a combination of inputs and outputs).
 */
 
class TCA8418 : public GPIOBase<uint64_t> {
public:
  static void create(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (checkNoOverlap(vpin, nPins, i2cAddress))
      // temporarily use the simple 18-pin GPIO mode - we'll switch to 8x8 matrix once this works
      new TCA8418(vpin, (nPins = (nPins > 18) ? 18 : nPins), i2cAddress, interruptPin);
  }

private:  
  // Constructor
  TCA8418(VPIN vpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) 
    : GPIOBase<uint64_t>((FSH *)F("TCA8418"), vpin, nPins, i2cAddress, interruptPin) 
  {
    uint8_t receiveBuffer[1];
    uint8_t commandBuffer[1];
    uint8_t status;

    commandBuffer[0] = REG_INT_STAT;  // Check interrupt status
    status = I2CManager.read(_I2CAddress, receiveBuffer, sizeof(receiveBuffer), commandBuffer, sizeof(commandBuffer));
    if (status == I2C_STATUS_OK) {
      DIAG(F("TCA8418 Interrupt status was: %x"), receiveBuffer[0]);
    }
    else
      DIAG(F("TCA8418 Interrupt status failed to read!"));
    // requestBlock.setRequestParams(_I2CAddress, inputBuffer, sizeof(inputBuffer),
    //   outputBuffer, sizeof(outputBuffer));
    // outputBuffer[0] = REG_GPIOA;
  }
  void _writeGpioPort() override {
    // I2CManager.write(_I2CAddress, 3, REG_GPIOA, _portOutputState, _portOutputState>>8);
  }
  void _writePullups() override {
    // Set pullups only for in-use pins.  This prevents pullup being set for a pin that
    //  is intended for use as an output but hasn't been written to yet.
    uint32_t temp = _portPullup & _portInUse;
    // I2CManager.write(_I2CAddress, 3, REG_GPPUA, temp, temp>>8);  
  }
  void _writePortModes() override {
    // Write 0 to each GPIO_DIRn for in-use pins that are inputs, 1 for outputs
    uint64_t temp = _portMode & _portInUse;
    DIAG(F("TCA8418 writing Port Mode: %x, to GPIO_DIRs"), temp);
    DIAG(F("TCA8418 writing Port Mode: %x, to GPIO_DIR1"), (temp&0xFF));
    I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR1, (temp&0xFF));
    DIAG(F("TCA8418 writing Port Mode: %x, to GPIO_DIR2"), ((temp&0xFF00)>>8));
    I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR2, ((temp&0xFF00)>>8));
    DIAG(F("TCA8418 writing Port Mode: %x, to GPIO_DIR3"), (temp&0x30000)>>16);
    I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR3, ((temp&0x30000)>>16));

    // Enable interrupt for in-use pins which are inputs (_portMode=0)
    // TCA8418 has interrupt enables per pin, but must be configured for low->high
    // or high->low... unlike the MCP23017
    temp = ~_portMode & _portInUse;
    DIAG(F("TCA8418 writing interrupt Port Mode: %x, to GPIO_INT_ENs"), temp);
    DIAG(F("TCA8418 writing interrupt Port Mode: %x, to GPIO_INT_EN1"), (temp&0xFF));
    I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN1, (temp&0xFF));
    DIAG(F("TCA8418 writing interrupt Port Mode: %x, to GPIO_INT_EN2"), ((temp&0xFF00)>>8));
    I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN2, ((temp&0xFF00)>>8));
    DIAG(F("TCA8418 writing interrupt Port Mode: %x, to GPIO_INT_EN3"), (temp&0x30000)>>16);
    I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN3, ((temp&0x30000)>>16));
    // I2CManager.write(_I2CAddress, 3, REG_INTCONA, 0x00, 0x00);
    // I2CManager.write(_I2CAddress, 3, REG_GPINTENA, temp, temp>>8);
  }
  void _readGpioPort(bool immediate) override {
    // if (immediate) {
    //   uint8_t buffer[2];
    //   I2CManager.read(_I2CAddress, buffer, 2, 1, REG_GPIOA);
    //   _portInputState = ((uint16_t)buffer[1]<<8) | buffer[0] | _portMode;
    // } else {
    //   // Queue new request
    //   requestBlock.wait(); // Wait for preceding operation to complete
    //   // Issue new request to read GPIO register
    //   I2CManager.queueRequest(&requestBlock);
    // }
  }
  // This function is invoked when an I/O operation on the requestBlock completes.
  void _processCompletion(uint8_t status) override {
    // if (status == I2C_STATUS_OK) 
    //   _portInputState = (((uint16_t)inputBuffer[1]<<8) | inputBuffer[0]) | _portMode;
    // else  
    //   _portInputState = 0xffff;
  }

  void _setupDevice() override {
    DIAG(F("TCA8418 setupDevice() called"));
    // IOCON is set MIRROR=1, ODR=1 (open drain shared interrupt pin)
    // I2CManager.write(_I2CAddress, 2, REG_IOCON, 0x44);
    _writePortModes();
    _writePullups();
    _writeGpioPort();
  }
 
  enum
  {
    REG_FIRST_RESERVED = 0x00,
    REG_CFG = 0x01,
    REG_INT_STAT = 0x02,
    REG_KEY_LCK_EC = 0x03,
    REG_KEY_EVENT_A = 0x04,
    REG_KEY_EVENT_B = 0x05,
    REG_KEY_EVENT_C = 0x06,
    REG_KEY_EVENT_D = 0x07,
    REG_KEY_EVENT_E = 0x08,
    REG_KEY_EVENT_F = 0x09,
    REG_KEY_EVENT_G = 0x0A,
    REG_KEY_EVENT_H = 0x0B,
    REG_KEY_EVENT_I = 0x0C,
    REG_KEY_EVENT_J = 0x0D,
    REG_KP_LCK_TIMER = 0x0E,
    REG_UNLOCK1 = 0x0F,
    REG_UNLOCK2 = 0x10,
    REG_GPIO_INT_STAT1 = 0x11,
    REG_GPIO_INT_STAT2 = 0x12,
    REG_GPIO_INT_STAT3 = 0x13,
    REG_GPIO_DAT_STAT1 = 0x14,
    REG_GPIO_DAT_STAT2 = 0x15,
    REG_GPIO_DAT_STAT3 = 0x16,
    REG_GPIO_DAT_OUT1 = 0x17,
    REG_GPIO_DAT_OUT2 = 0x18,
    REG_GPIO_DAT_OUT3 = 0x19,
    REG_GPIO_INT_EN1 = 0x1A,
    REG_GPIO_INT_EN2 = 0x1B,
    REG_GPIO_INT_EN3 = 0x1C,
    REG_KP_GPIO1 = 0x1D,
    REG_KP_GPIO2 = 0x1E,
    REG_KP_GPIO3 = 0x1F,
    REG_GPI_EM1 = 0x20,
    REG_GPI_EM2 = 0x21,
    REG_GPI_EM3 = 0x22,
    REG_GPIO_DIR1 = 0x23,
    REG_GPIO_DIR2 = 0x24,
    REG_GPIO_DIR3 = 0x25,
    REG_GPIO_INT_LVL1 = 0x26,
    REG_GPIO_INT_LVL2 = 0x27,
    REG_GPIO_INT_LVL3 = 0x28,
    REG_DEBOUNCE_DIS1 = 0x29,
    REG_DEBOUNCE_DIS2 = 0x2A,
    REG_DEBOUNCE_DIS3 = 0x2B,
    REG_GPIO_PULL1 = 0x2C,
    REG_GPIO_PULL2 = 0x2D,
    REG_GPIO_PULL3 = 0x2E,
    REG_LAST_RESERVED = 0x2F,
  };
};

#endif