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
 * The device driver allocates up to 3 vpins to the device.  A digital read on the first pin
 * will return a value that indicates whether the object is within the threshold range (1)
 * or not (0).  An analogue read on the first pin returns the last measured distance (in mm), 
 * the second pin returns the signal strength, and the third pin returns detected 
 * ambient light level.  By default the device takes around 60ms to complete a ranging 
 * operation, so we do a 100ms cycle (10 samples per second).
 * 
 * The VL53L0X is initially set to respond to I2C address 0x29.  If you only have one module,
 * you can use this address.  However, the address can be modified by software.  If
 * you select another address, that address will be written to the device and used until the device is reset.
 * 
 * If you have more than one module, then you will need to specify a digital VPIN (Arduino
 * digital output or I/O extender pin) which you connect to the module's XSHUT pin.  Now,
 * when the device driver starts, the XSHUT pin is set LOW to turn the module off.  Once 
 * all VL53L0X modules are turned off, the driver works through each module in turn,
 * setting XSHUT to HIGH to turn that module on, then writing that module's desired I2C address.
 * In this way, many VL53L0X modules can be connected to the one I2C bus, each one 
 * using a distinct I2C address.  The process is described in ST Microelectronics application
 * note AN4846.
 * 
 * WARNING:  If the device's XSHUT pin is not connected, then it is very prone to noise, 
 * and the device may reset spontaneously or when handled and the device will stop responding
 * on its allocated address.  If you're not using XSHUT, then tie it to +5V via a resistor 
 * (should be tied to +2.8V strictly). Some manufacturers (Adafruit and Polulu for example)
 * include a pull-up on the module, but others don't.
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
 *       XSHUT terminal on the module.  The digital output may be an Arduino pin or an
 *       I/O extender pin.
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
  uint8_t _nextState = STATE_INIT;
  I2CRB _rb;
  uint8_t _inBuffer[12];
  uint8_t _outBuffer[2];
  static bool _addressConfigInProgress;

  // State machine states.
  enum : uint8_t {
    STATE_INIT = 0,
    STATE_RESTARTMODULE = 1,
    STATE_CONFIGUREADDRESS = 2,
    STATE_SKIP = 3,
    STATE_CONFIGUREDEVICE = 4,
    STATE_INITIATESCAN = 5,
    STATE_CHECKSTATUS = 6,
    STATE_GETRESULTS = 7,
    STATE_DECODERESULTS = 8,
    STATE_FAILED = 9,
  };

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
  static void create(VPIN firstVpin, int nPins, uint8_t i2cAddress, uint16_t onThreshold, uint16_t offThreshold, VPIN xshutPin = VPIN_NONE) {
     if (checkNoOverlap(firstVpin, nPins,i2cAddress)) new VL53L0X(firstVpin, nPins, i2cAddress, onThreshold, offThreshold, xshutPin);
  }

protected:
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
  void _begin() override {
    // If there's only one device, then the XSHUT pin need not be connected.  However, 
    //  the device will not respond on its default address if it has 
    //  already been changed.  Therefore, we skip the address configuration if the 
    //  desired address is already responding on the I2C bus.
    if (_xshutPin == VPIN_NONE && I2CManager.exists(_i2cAddress)) {
      // Device already present on this address, so skip the address initialisation.
      _nextState = STATE_CONFIGUREDEVICE;
    }
  }

  void _loop(unsigned long currentMicros) override {
    uint8_t status;
    switch (_nextState) {
      case STATE_INIT:
        // On first entry to loop, reset this module by pulling XSHUT low.  Each module
        // will be addressed in turn, until all are in the reset state.
        // If no XSHUT pin is configured, then only one device is supported.
        if (_xshutPin != VPIN_NONE) IODevice::write(_xshutPin, 0);
        _nextState = STATE_RESTARTMODULE;
        delayUntil(currentMicros+1000);
        break;
      case STATE_RESTARTMODULE:
        // On second entry, set XSHUT pin high to allow this module to restart.
        // On some modules, there is a diode in series with the XSHUT pin to 
        // protect the low-voltage pin against +5V, but we can provide additional
        // protection by enabling the pull-up resistor on the microcontroller
        // instead of driving the output directly.
        // Ensure XSHUT is set for only one module at a time by using a
        // shared flag accessible to all device instances.
        if (_addressConfigInProgress) return;
        _addressConfigInProgress = true;
        // Set XSHUT pin (if connected).  Because of supply voltage differences,
        // drive the signal through the digital output's pull-up resistor.
        if (_xshutPin != VPIN_NONE) IODevice::configureInput(_xshutPin, 1);
        // Allow the module time to restart
        delayUntil(currentMicros+10000);
        _nextState = STATE_CONFIGUREADDRESS;
        break;
      case STATE_CONFIGUREADDRESS:
        // Then write the desired I2C address to the device, while this is the only
        //  module responding to the default address.
        I2CManager.write(VL53L0X_I2C_DEFAULT_ADDRESS, 2, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, _i2cAddress);
        _addressConfigInProgress = false;
        _nextState = STATE_SKIP;
        break;
      case STATE_SKIP:
        // Do nothing on the third entry.
        _nextState = STATE_CONFIGUREDEVICE;
        break;
      case STATE_CONFIGUREDEVICE:
        // On next entry, check if device address has been set.
        if (I2CManager.exists(_i2cAddress)) {
          #ifdef DIAG_IO
          _display();
          #endif
          // Set 2.8V mode
          write_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
            read_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
          _nextState = STATE_INITIATESCAN;
        } else {
          DIAG(F("VL53L0X I2C:x%x device not responding"), _i2cAddress);
          _deviceState = DEVSTATE_FAILED;
          _nextState = STATE_FAILED;
        }
        break;
      case STATE_INITIATESCAN:
        // Not scanning, so initiate a scan
        _outBuffer[0] = VL53L0X_REG_SYSRANGE_START;
        _outBuffer[1] = 0x01;
        I2CManager.write(_i2cAddress, _outBuffer, 2, &_rb);
        _nextState = STATE_CHECKSTATUS;
        break;
      case STATE_CHECKSTATUS:
        status = _rb.status;
        if (status == I2C_STATUS_PENDING) return; // try next time
        if (status != I2C_STATUS_OK) {
          reportError(status);
          _nextState = STATE_FAILED;
        } else
          _nextState = STATE_GETRESULTS;
        delayUntil(currentMicros + 95000); // wait for 95 ms before checking.
        break;
      case STATE_GETRESULTS:
        // Ranging completed.  Request results
        _outBuffer[0] = VL53L0X_REG_RESULT_RANGE_STATUS;
        I2CManager.read(_i2cAddress, _inBuffer, 12, _outBuffer, 1, &_rb);
        _nextState = 3;
        delayUntil(currentMicros + 5000); // Allow 5ms to get data
        _nextState = STATE_DECODERESULTS;
        break;
      case STATE_DECODERESULTS:
        // If I2C write still busy, return.
        status = _rb.status;
        if (status == I2C_STATUS_PENDING) return; // try again next time
        if (status == I2C_STATUS_OK) {
          if (!(_inBuffer[0] & 1)) return; // device still busy
          uint8_t deviceRangeStatus = ((_inBuffer[0] & 0x78) >> 3);
          if (deviceRangeStatus == 0x0b) {
            // Range status OK, so use data
            _ambient = makeuint16(_inBuffer[7], _inBuffer[6]);
            _signal = makeuint16(_inBuffer[9], _inBuffer[8]);
            _distance = makeuint16(_inBuffer[11], _inBuffer[10]);
            if (_distance <= _onThreshold) 
              _value = true;
            else if (_distance > _offThreshold) 
              _value = false;
          }
          // Completed. Restart scan on next loop entry.
          _nextState = STATE_INITIATESCAN;
        } else {
          reportError(status);
          _nextState = STATE_FAILED;
        }
        break;
      case STATE_FAILED:
        // Do nothing.
        delayUntil(currentMicros+1000000UL);
        break;
      default:
        break;
    }
  }

  // Function to report a failed I2C operation.  Put the device off-line.
  void reportError(uint8_t status) {
    DIAG(F("VL53L0X I2C:x%x Error:%d %S"), _i2cAddress, status, I2CManager.getErrorMessage(status));
    _deviceState = DEVSTATE_FAILED;
    _value = false;
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

  // For digital read, return zero for all but first pin.
  int _read(VPIN vpin) override {
    if (vpin == _firstVpin)
      return _value;
    else
      return 0;
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
    // read byte from register and return value
    I2CManager.read(_i2cAddress, _inBuffer, 1, &reg, 1);
    return _inBuffer[0];
  }
};

bool VL53L0X::_addressConfigInProgress = false;

#endif // IO_VL53L0X_h
