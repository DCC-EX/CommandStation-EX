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
 * WARNING:  If the device's XSHUT pin is not connected, then it may be prone to noise, 
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
    STATE_INIT,
    STATE_RESTARTMODULE,
    STATE_CONFIGUREADDRESS,
    STATE_CONFIGUREDEVICE,
    STATE_INITIATESCAN,
    STATE_CHECKSTATUS,
    STATE_GETRESULTS,
    STATE_DECODERESULTS,
    STATE_FAILED,
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
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint16_t onThreshold, uint16_t offThreshold, VPIN xshutPin = VPIN_NONE) {
     if (checkNoOverlap(firstVpin, nPins,i2cAddress)) new VL53L0X(firstVpin, nPins, i2cAddress, onThreshold, offThreshold, xshutPin);
  }

protected:
  VL53L0X(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint16_t onThreshold, uint16_t offThreshold, VPIN xshutPin = VPIN_NONE) {
    _firstVpin = firstVpin;
    _nPins = (nPins > 3) ? 3 : nPins;
    _I2CAddress = i2cAddress;
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
    _nextState = STATE_INIT;
    _addressConfigInProgress = false;
  }

  void _loop(unsigned long currentMicros) override {
    uint8_t status;
    switch (_nextState) {
      case STATE_INIT:
        if (I2CManager.exists(_I2CAddress)) {
          // Device already present on the nominated address, so skip the address initialisation.
          _nextState = STATE_CONFIGUREDEVICE;
        } else {
          // On first entry to loop, reset this module by pulling XSHUT low.  Each module
          // will be addressed in turn, until all are in the reset state.
          // If no XSHUT pin is configured, then only one device is supported.
          if (_xshutPin != VPIN_NONE) IODevice::write(_xshutPin, 0);
          _nextState = STATE_RESTARTMODULE;
          delayUntil(currentMicros+10000);
        }
        break;
      case STATE_RESTARTMODULE:
        // On second entry, set XSHUT pin high to allow this module to restart.
        // I've observed that the device tends to randomly reset if the XSHUT 
        // pin is set high from a 5V arduino, even through a pullup resistor.  
        // Assume that there will be a pull-up on the XSHUT pin to +2.8V as
        // recommended in the device datasheet.  Then we only need to
        // turn our output pin high-impedence (by making it an input) and the
        // on-board pullup will do its job.
        // Ensure XSHUT is set for only one module at a time by using a
        // shared flag accessible to all device instances.
        if (!_addressConfigInProgress) {
          _addressConfigInProgress = true;
          // Configure XSHUT pin (if connected) to bring the module out of sleep mode.
          if (_xshutPin != VPIN_NONE) IODevice::configureInput(_xshutPin, false);
          // Allow the module time to restart
          delayUntil(currentMicros+10000);
          _nextState = STATE_CONFIGUREADDRESS;
        }
        break;
      case STATE_CONFIGUREADDRESS:
        // Then write the desired I2C address to the device, while this is the only
        //  module responding to the default address.
        {
          #if defined(I2C_EXTENDED_ADDRESS)
          // Add subbus reference for desired address to the device default address.
          I2CAddress defaultAddress = {_I2CAddress, VL53L0X_I2C_DEFAULT_ADDRESS};
          status = I2CManager.write(defaultAddress, 2, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, _I2CAddress.deviceAddress());
          #else
          status = I2CManager.write(VL53L0X_I2C_DEFAULT_ADDRESS, 2, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, _I2CAddress);
          #endif
          if (status != I2C_STATUS_OK) {
            reportError(status);
          }
        }
        delayUntil(currentMicros+10000);
        _nextState = STATE_CONFIGUREDEVICE;
        break;
      case STATE_CONFIGUREDEVICE:
        // Allow next VL53L0X device to be configured
        _addressConfigInProgress = false;
        // Now check if device address has been set.
        if (I2CManager.exists(_I2CAddress)) {
          #ifdef DIAG_IO
          _display();
          #endif
          // Set 2.8V mode
          status = write_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
            read_reg(VL53L0X_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
          if (status != I2C_STATUS_OK) {
            reportError(status);
          } else
            _nextState = STATE_INITIATESCAN;
        } else {
          DIAG(F("VL53L0X I2C:%s device not responding"), _I2CAddress.toString());
          _deviceState = DEVSTATE_FAILED;
          _nextState = STATE_FAILED;
        }
        break;
      case STATE_INITIATESCAN:
        // Not scanning, so initiate a scan
        _outBuffer[0] = VL53L0X_REG_SYSRANGE_START;
        _outBuffer[1] = 0x01;
        I2CManager.write(_I2CAddress, _outBuffer, 2, &_rb);
        _nextState = STATE_CHECKSTATUS;
        break;
      case STATE_CHECKSTATUS:
        status = _rb.status;
        if (status == I2C_STATUS_PENDING) return; // try next time
        if (status != I2C_STATUS_OK) {
          reportError(status);
        } else
          _nextState = STATE_GETRESULTS;
        delayUntil(currentMicros + 95000); // wait for 95 ms before checking.
        break;
      case STATE_GETRESULTS:
        // Ranging completed.  Request results
        _outBuffer[0] = VL53L0X_REG_RESULT_RANGE_STATUS;
        I2CManager.read(_I2CAddress, _inBuffer, 12, _outBuffer, 1, &_rb);
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

  // Function to report a failed I2C operation.
  void reportError(uint8_t status) {
    DIAG(F("VL53L0X I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
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
    DIAG(F("VL53L0X I2C:%s Configured on Vpins:%u-%u On:%dmm Off:%dmm %S"),
      _I2CAddress.toString(), _firstVpin, _firstVpin+_nPins-1, _onThreshold, _offThreshold,
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
    return I2CManager.write(_I2CAddress, outBuffer, 2);
  }
  uint8_t read_reg(uint8_t reg) {
    // read byte from register and return value
    I2CManager.read(_I2CAddress, _inBuffer, 1, &reg, 1);
    return _inBuffer[0];
  }
};

bool VL53L0X::_addressConfigInProgress = false;

#endif // IO_VL53L0X_h
