/*
 * © 2026. All rights reserved.
 *
 * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * File EXRAILMacros.h line 674-691 have the ANOUT aliases for the LSS commands defined
 *
 */

#ifndef IO_LSS_h
#define IO_LSS_h

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

class IO_LSS : public IODevice {
public:
  enum LSSCmd {
    LSS_CMD_LOAD = 1,
    LSS_CMD_FLUSH = 2,
    LSS_CMD_PLAY = 3,
    LSS_CMD_PLAY_LOOP = 4,
    LSS_CMD_STOP = 5,
    LSS_CMD_PAUSE = 6,
    LSS_CMD_RESUME = 7,
    LSS_CMD_VOLUME = 8,
    LSS_CMD_FADE = 9,
    LSS_CMD_GLOBAL_RESET = 10,
    LSS_CMD_GLOBAL_MUTE = 11,
    LSS_CMD_OLED_PAGE = 12,
    LSS_CMD_RUN_SCRIPT = 13,
    LSS_CMD_STOP_SCRIPT = 14
  };

  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) {
      new IO_LSS(firstVpin, nPins, i2cAddress);
    }
  }

private:
  IO_LSS(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress) : IODevice(firstVpin, nPins) {
    _I2CAddress = i2cAddress;
    addDevice(this);
  }

  void _begin() override {
    I2CManager.begin();
    if (!I2CManager.exists(_I2CAddress)) {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("LSS error: Device not found at I2C address %s"), _I2CAddress.toString());
      return;
    }

    uint8_t sysStatusReg = 0x02;
    uint8_t sysStatusVal = 0;
    if (I2CManager.read(_I2CAddress, &sysStatusVal, 1, &sysStatusReg, 1) == I2C_STATUS_OK) {
      DIAG(F("LSS found at address %s, SYS_STATUS: 0x%02X"), _I2CAddress.toString(), sysStatusVal);
      _deviceState = DEVSTATE_NORMAL;
    } else {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("LSS error: Failed to read SYS_STATUS from address %s"), _I2CAddress.toString());
    }
  }

  void _loop(unsigned long currentMicros) override {
    delayUntil(currentMicros + 100000UL); // Run loop every 100ms
  }
};

#endif
