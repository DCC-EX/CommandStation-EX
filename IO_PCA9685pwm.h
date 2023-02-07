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
 * This driver performs the basic interface between the HAL and an 
 * I2C-connected PCA9685 16-channel PWM module.  When requested, it 
 * commands the device to set the PWM mark-to-period ratio accordingly.
 * The call to IODevice::writeAnalogue(vpin, value) specifies the
 * desired value in the range 0-4095 (0=0% and 4095=100%).
 */

#ifndef PCA9685_BASIC_H
#define PCA9685_BASIC_H

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

/*
 * IODevice subclass for PCA9685 16-channel PWM module.
 */
 
class PCA9685pwm : public IODevice {
public:
  // Create device driver instance.
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new PCA9685pwm(firstVpin, nPins, i2cAddress);
  }

private:
  
  // structures for setting up non-blocking writes to servo controller
  I2CRB requestBlock;
  uint8_t outputBuffer[5];

  // REGISTER ADDRESSES
  const uint8_t PCA9685_MODE1=0x00;      // Mode Register 
  const uint8_t PCA9685_FIRST_SERVO=0x06;  /** low uint8_t first servo register ON*/
  const uint8_t PCA9685_PRESCALE=0xFE;     /** Prescale register for PWM output frequency */
  // MODE1 bits
  const uint8_t MODE1_SLEEP=0x10;   /**< Low power mode. Oscillator off */
  const uint8_t MODE1_AI=0x20;      /**< Auto-Increment enabled */
  const uint8_t MODE1_RESTART=0x80; /**< Restart enabled */

  const float FREQUENCY_OSCILLATOR=25000000.0; /** Accurate enough for our purposes  */
  const uint8_t PRESCALE_50HZ = (uint8_t)(((FREQUENCY_OSCILLATOR / (50.0 * 4096.0)) + 0.5) - 1);
  const uint32_t MAX_I2C_SPEED = 1000000L; // PCA9685 rated up to 1MHz I2C clock speed

  // Constructor
  PCA9685pwm(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    _nPins = (nPins>16) ? 16 : nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);

    // Initialise structure used for setting pulse rate
    requestBlock.setWriteParams(_I2CAddress, outputBuffer, sizeof(outputBuffer));
  }

  // Device-specific initialisation
  void _begin() override {
    I2CManager.begin();
    I2CManager.setClock(1000000); // Nominally able to run up to 1MHz on I2C
            // In reality, other devices including the Arduino will limit 
            // the clock speed to a lower rate.

    // Initialise I/O module here.
    if (I2CManager.exists(_I2CAddress)) {
      writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
      writeRegister(_I2CAddress, PCA9685_PRESCALE, PRESCALE_50HZ);   // 50Hz clock, 20ms pulse period.
      writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_AI);
      writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
      // In theory, we should wait 500us before sending any other commands to each device, to allow
      // the PWM oscillator to get running.  However, we don't do any specific wait, as there's 
      // plenty of other stuff to do before we will send a command.
    #if defined(DIAG_IO)
      _display();
    #endif
    } else
      _deviceState = DEVSTATE_FAILED;
  }

  // Device-specific writeAnalogue function, invoked from IODevice::writeAnalogue().
  //            
  void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override {
    (void)param1; (void)param2;  // suppress compiler warning
    #ifdef DIAG_IO
    DIAG(F("PCA9685pwm WriteAnalogue Vpin:%d Value:%d Profile:%d Duration:%d %S"), 
      vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
    #endif
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    if (value > 4095) value = 4095;
    else if (value < 0) value = 0;

    writeDevice(pin, value);
  }

  // Display details of this device.
  void _display() override {
    DIAG(F("PCA9685pwm I2C:x%x Configured on Vpins:%d-%d %S"), (int)_I2CAddress, (int)_firstVpin, 
      (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }

  // writeDevice (helper function) takes a pin in range 0 to _nPins-1 within the device, and a value
  // between 0 and 4095 for the PWM mark-to-period ratio, with 4095 being 100%.
  void writeDevice(uint8_t pin, int value) {
    #ifdef DIAG_IO
    DIAG(F("PCA9685pwm I2C:x%x WriteDevice Pin:%d Value:%d"), (int)_I2CAddress, pin, value);
    #endif
    // Wait for previous request to complete
    uint8_t status = requestBlock.wait();
    if (status != I2C_STATUS_OK) {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("PCA9685pwm I2C:x%x failed %S"), (int)_I2CAddress, I2CManager.getErrorMessage(status));
    } else {
      // Set up new request.
      outputBuffer[0] = PCA9685_FIRST_SERVO + 4 * pin;
      outputBuffer[1] = 0;
      outputBuffer[2] = (value == 4095 ? 0x10 : 0);  // 4095=full on
      outputBuffer[3] = value & 0xff;
      outputBuffer[4] = value >> 8;
      I2CManager.queueRequest(&requestBlock);
    }
  }

  // Internal helper function for this device
  static void writeRegister(I2CAddress address, uint8_t reg, uint8_t value) {
    I2CManager.write(address, 2, reg, value);
  }

};

#endif