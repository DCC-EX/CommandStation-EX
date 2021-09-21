/*
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

/*
 * The VL53L0X Time-Of-Flight sensor operates by sending a short laser pulse and detecting
 * the reflection of the pulse.  The time between the pulse and the receipt of reflections
 * is measured and used to determine the distance to the reflecting object.
 * 
 * For economy of memory and processing time, this driver includes only part of the code 
 * that ST provide in their API.  Also, the API code isn't very clear and it is not easy
 * to identify what operations are useful and what are not. 
 * The operation shown here doesn't include any calibration, so is probably not as accurate
 * as using the full driver, but it's probably accurate enough for the purpose.
 * 
 * The device driver allocates up to 3 vpins to the device.  A digital read on any of the pins
 * will return a value that indicates whether the object is within the threshold range (1)
 * or not (0).  An analogue read on the first pin returns the last measured distance (in mm), 
 * the second pin returns the signal strength, and the third pin returns detected 
 * ambient light level.
 * 
 * The VL53L0X is initially set to respond to I2C address 0x29.  If you only have one module,
 * you can use this address.  However, the address can be modified by software.  If
 * you select another address, that address will be written to the device and used until the device is reset.
 * 
 * If you have more than one module, then you will need to specify a digital VPIN (Arduino
 * digital output or I/O extender pin) which you connect to the module's XSHUT pin.  Now,
 * when the device driver starts, the XSHUT pin is set LOW to turn the module off.  Once 
 * all VL53L0X modules are turned off, the driver works through each module in turn by
 * setting XSHUT to HIGH to turn the module on,, then writing the module's desired I2C address.
 * In this way, many VL53L0X modules can be connected to the one I2C bus, each one 
 * using a distinct I2C address.
 * 
 * WARNING:  If the device's XSHUT pin is not connected, then it is very prone to noise, 
 * and the device may even reset when handled.  If you're not using XSHUT, then it's 
 * best to tie it to +5V.
 * 
 * The driver is configured as follows:
 * 
 * Single VL53L0X module:
 *      VL53L0X::create(firstVpin, nPins, i2cAddress, lowThreshold, highThreshold);
 * Where firstVpin is the first vpin reserved for reading the device,
 *       nPins is 1, 2 or 3,
 *       i2cAddress is the address of the device (normally 0x29),
 *       lowThreshold is the distance at which the digital vpin state is set to 1 (in mm),
 *   and highThreshold is the distance at which the digital vpin state is set to 0 (in mm).
 * 
 * Multiple VL53L0X modules:
 *       VL53L0X::create(firstVpin, nPins, i2cAddress, lowThreshold, highThreshold, xshutPin);
 *       ...
 * Where firstVpin is the first vpin reserved for reading the device,
 *       nPins is 1, 2 or 3,
 *       i2cAddress is the address of the device (any valid address except 0x29),
 *       lowThreshold is the distance at which the digital vpin state is set to 1 (in mm),
 *       highThreshold is the distance at which the digital vpin state is set to 0 (in mm),
 *   and xshutPin is the VPIN number corresponding to a digital output that is connected to the
 *       XSHUT terminal on the module.
 * 
 * Example:
 *   In mySetup function within mySetup.cpp:
 *      VL53L0X::create(4000, 3, 0x29, 200, 250);
 *      Sensor::create(4000, 4000, 0);  // Create a sensor
 * 
 *   When an object comes within 200mm of the sensor, a message 
 *      <Q 4000>
 *   will be sent over the serial USB, and when the object moves more than 250mm from the sensor, 
 *   a message
 *      <q 4000>
 *   will be sent. 
 * 
 */

#ifndef IO_VL53L0X_h
#define IO_VL53L0X_h

#include "IODevice.h"

class VL53L0X : public IODevice {
private: 
  uint8_t _i2cAddress;
  uint16_t _ambient;
  uint16_t _distance;
  uint16_t _signal;
  uint16_t _onThreshold;
  uint16_t _offThreshold;
  VPIN _xshutPin;
  bool _value;
  bool _initialising = true;
  uint8_t _entryCount = 0;
  bool _scanInProgress = false;
  // Register addresses
  enum : uint8_t {
    VL53L0X_REG_SYSRANGE_START=0x00,
    VL53L0X_REG_RESULT_INTERRUPT_STATUS=0x13,
    VL53L0X_REG_RESULT_RANGE_STATUS=0x14,
    VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV=0x89,
    VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS=0x8A,
  };
  const uint8_t VL53L0X_I2C_DEFAULT_ADDRESS=0x29;

public:
  VL53L0X(VPIN firstVpin, int nPins, uint8_t i2cAddress, uint16_t onThreshold, uint16_t offThreshold, VPIN xshutPin = VPIN_NONE) {
    _firstVpin = firstVpin;
    _nPins = min(nPins, 3);
    _i2cAddress = i2cAddress;
    _onThreshold = onThreshold;
    _offThreshold = offThreshold;
    _xshutPin = xshutPin;
    _value = 0;
    addDevice(this);
  }
  static void create(VPIN firstVpin, int nPins, uint8_t i2cAddress, uint16_t onThreshold, uint16_t offThreshold, VPIN xshutPin = VPIN_NONE) {
    new VL53L0X(firstVpin, nPins, i2cAddress, onThreshold, offThreshold, xshutPin);
  }

protected:
  void _begin() override {
    _initialising = true;
    // Check if device is already responding on the nominated address.
    if (I2CManager.exists(_i2cAddress)) {
      // Yes, it's already on this address, so skip the address initialisation.
      _entryCount = 3;
    } else {
      _entryCount = 0;
    }
  }

  void _loop(unsigned long currentMicros) override {
    if (_initialising) {
      switch (_entryCount++) {
        case 0:
          // On first entry to loop, reset this module by pulling XSHUT low.  All modules
          // will be reset in turn.
          if (_xshutPin != VPIN_NONE) IODevice::write(_xshutPin, 0);
          break;
        case 1:
          // On second entry, set XSHUT pin high to allow the module to restart.
          // On the module, there is a diode in series with the XSHUT pin to 
          // protect the low-voltage pin against +5V.
          if (_xshutPin != VPIN_NONE) IODevice::write(_xshutPin, 1);
          // Allow the module time to restart
          delay(10);
          // Then write the desired I2C address to the device, while this is the only
          //  module responding to the default address.
          I2CManager.write(VL53L0X_I2C_DEFAULT_ADDRESS, 2, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, _i2cAddress);
          break;
        case 3:
          // After two more loops, check if device has been configured.
          if (I2CManager.exists(_i2cAddress)) {
            #ifdef DIAG_IO
            _display();
            #endif
            // Set 2.8V mode
            write_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
              read_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
          } else {
            DIAG(F("VL53L0X I2C:x%x device not responding"), _i2cAddress);
            _deviceState = DEVSTATE_FAILED;
          }
          _initialising = false;
          _entryCount = 0;
          break;
        default:
          break;
      }
    } else {

      if (!_scanInProgress) {
        // Not scanning, so initiate a scan
        uint8_t status = write_reg(VL53L0X_REG_SYSRANGE_START, 0x01);
        if (status != I2C_STATUS_OK) {
          DIAG(F("VL53L0X I2C:x%x Error:%d %S"), _i2cAddress, status, I2CManager.getErrorMessage(status));
          _deviceState = DEVSTATE_FAILED;
          _value = false;
        } else
          _scanInProgress = true;

      } else {
        // Scan in progress, so check for completion.
        uint8_t status = read_reg(VL53L0X_REG_RESULT_RANGE_STATUS);
        if (status & 1) {
          // Completed.  Retrieve data
          uint8_t inBuffer[12];
          read_registers(VL53L0X_REG_RESULT_RANGE_STATUS, inBuffer, 12);
          uint8_t deviceRangeStatus = ((inBuffer[0] & 0x78) >> 3);
          if (deviceRangeStatus == 0x0b) {
            // Range status OK, so use data
            _ambient = makeuint16(inBuffer[7], inBuffer[6]);
            _signal = makeuint16(inBuffer[9], inBuffer[8]);
            _distance = makeuint16(inBuffer[11], inBuffer[10]);
            if (_distance <= _onThreshold) 
              _value = true;
            else if (_distance > _offThreshold) 
              _value = false;
          }
          _scanInProgress = false;
        }
      }
      // Next entry in 10 milliseconds.
      delayUntil(currentMicros + 10000UL);
    }
  }

  // For analogue read, first pin returns distance, second pin is signal strength, and third is ambient level.
  int _readAnalogue(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    switch (pin) {
      case 0:
        return _distance;
      case 1:
        return _signal;
      case 2:
        return _ambient;
      default:
        return -1;
    }
  }

  // For digital read, return the same value for all pins.
  int _read(VPIN) override {
    return _value;
  }

  void _display() override {
    DIAG(F("VL53L0X I2C:x%x Configured on Vpins:%d-%d On:%dmm Off:%dmm %S"),
      _i2cAddress, _firstVpin, _firstVpin+_nPins-1, _onThreshold, _offThreshold,
      (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
  

private:
  inline uint16_t makeuint16(byte lsb, byte msb) {
    return (((uint16_t)msb) << 8) | lsb;
  }
  uint8_t write_reg(uint8_t reg, uint8_t data) {
    // write byte to register
    uint8_t outBuffer[2];
    outBuffer[0] = reg;
    outBuffer[1] = data;
    return I2CManager.write(_i2cAddress, outBuffer, 2);
  }
  uint8_t read_reg(uint8_t reg) {
    // read byte from register register
    uint8_t inBuffer[1];
    I2CManager.read(_i2cAddress, inBuffer, 1, &reg, 1);
    return inBuffer[0];
  }
  void read_registers(uint8_t reg, uint8_t buffer[], uint8_t size) {
    I2CManager.read(_i2cAddress, buffer, size, &reg, 1);
  }
};

#endif // IO_VL53L0X_h
