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

//#define DEBUG_LSS_I2C // Uncomment to enable I2C read/write diagnostic logging

#include "IO_DFPlayerBase.h"
#include "I2CManager.h"
#include "DIAG.h"

class IO_LSS : public DFPlayerBase {
public:
  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) {
      new IO_LSS(firstVpin, nPins, i2cAddress);
    }
  }

private:
  // Global I2C Registers for LSS sound board
  enum LSS_GlobalRegister : uint8_t {
    REG_SLOT_SELECTOR   = 0x00, // Select active Slot (0-15)
    REG_GLOBAL_CMD      = 0x01, // Global commands (reset, mute, etc)
    REG_SYS_STATUS      = 0x02, // System status
    REG_FW_VER_MAJOR    = 0x03, // Firmware version MSB
    REG_FW_VER_MINOR    = 0x04, // Firmware version LSB
    REG_OLED_PAGE       = 0x05, // OLED page display and mode
    REG_CORE0_PEAK_LOAD = 0x06, // Core 0 peak load %
    REG_CORE1_PEAK_LOAD = 0x07, // Core 1 peak load %
    REG_SCRIPT_ENGINE   = 0x08, // Script engine selection (0-3)
    REG_SCRIPT_ID       = 0x09, // Script ID (16-bit write starts here)
    REG_SCRIPT_LSB      = 0x09, // Script ID LSB
    REG_SCRIPT_MSB      = 0x0A, // Script ID MSB
    REG_SCRIPT_CMD      = 0x0B, // Trigger script engine (start/stop)
    REG_TEMP_LSB        = 0x0C, // Internal temperature sensor LSB
    REG_TEMP_MSB        = 0x0D, // Internal temperature sensor MSB

    // Scripting Engine Telemetry bases
    REG_SCRIPT_ENG0_BASE = 0x30,
    REG_SCRIPT_ENG1_BASE = 0x38,
    REG_SCRIPT_ENG2_BASE = 0x40,
    REG_SCRIPT_ENG3_BASE = 0x48,

    // INA219 Power Monitor Telemetry
    REG_CURRENT_LSB     = 0x50, // INA219 Current LSB (16-bit, mA)
    REG_CURRENT_MSB     = 0x51, // INA219 Current MSB
    REG_VOLTAGE_LSB     = 0x52, // INA219 Bus Voltage LSB (16-bit, mV)
    REG_VOLTAGE_MSB     = 0x53, // INA219 Bus Voltage MSB
    REG_POWER_LSB       = 0x54, // INA219 Power LSB (16-bit, mW)
    REG_POWER_MSB       = 0x55  // INA219 Power MSB
  };

  // Pin offset definitions for virtual pins (0 to 31) relative to _firstVpin
  enum LSS_PinOffset : uint8_t {
    // 0 - 15: Audio Slots 0 - 15
    OFFSET_PIN_SLOT_BASE        = 0,
    OFFSET_PIN_SLOT_MAX         = 15,

    // 16 - 19: Script Engines 0 - 3 State (0=IDLE, non-zero=BUSY)
    OFFSET_PIN_SCRIPT_ENG_BASE  = 16,
    OFFSET_PIN_SCRIPT_ENG_MAX   = 19,

    // 20 - 23: Script Engines 0 - 3 Current Line Number (16-bit)
    OFFSET_PIN_SCRIPT_LINE_BASE = 20,
    OFFSET_PIN_SCRIPT_LINE_MAX  = 23,

    // 24 - 31: Global Telemetry & System
    OFFSET_PIN_SYS_STATUS       = 24, // 0x02 SYS_STATUS bitmask
    OFFSET_PIN_TEMP             = 25, // 0x0C TEMP (signed 16-bit, tenths of deg C)
    OFFSET_PIN_CURRENT          = 26, // 0x50 CURRENT (mA)
    OFFSET_PIN_VOLTAGE          = 27, // 0x52 VOLTAGE (mV)
    OFFSET_PIN_POWER            = 28, // 0x54 POWER (mW)
    OFFSET_PIN_CORE0_LOAD       = 29, // 0x06 Core 0 peak load %
    OFFSET_PIN_CORE1_LOAD       = 30, // 0x07 Core 1 peak load %
    OFFSET_PIN_OLED_PAGE        = 31  // 0x05 OLED page (R/W, 0-5)
  };

  // Indexed I2C Registers (Slot-specific)
  enum LSS_IndexedRegister : uint8_t {
    REG_PLAY_CONTROL    = 0x10, // Playback control (play, stop, pause, loop)
    REG_PLAY_STATUS     = 0x11, // Playback status
    REG_VOL_OUT_BASE    = 0x12, // Base volume control (REG_VOL_OUT_BASE + channel 0-7)
    REG_FILE_ID_LSB     = 0x20, // File selection LSB (16-bit write starts here)
    REG_FILE_ID_MSB     = 0x21, // File selection MSB
    REG_SLOT_COMMAND    = 0x22, // Load/Map or Flush commands
    REG_SLOT_STATUS     = 0x23, // Ready/Busy/Error state of the slot
    REG_SLOT_ERROR      = 0x24, // Detailed error bitmask
    REG_FILE_SAMPLE_RATE= 0x25, // Detected sample rate
    REG_FILE_BITRATE    = 0x26, // Detected bitrate (kbps)
    REG_FILE_CHANNELS   = 0x27, // Number of channels
    REG_FADE_CHANNEL    = 0x28, // Fade channel selection (0-7)
    REG_FADE_TARGET_VOL = 0x29, // Fade target volume (0-255)
    REG_FADE_DUR_LSB    = 0x2A, // Fade duration LSB (10mS increments, 16-bit write starts here)
    REG_FADE_DUR_MSB    = 0x2B, // Fade duration MSB (10mS increments)
    REG_FADE_TRIGGER    = 0x2C  // Trigger fade with curve (0-3)
  };

  // Scripting Engine Telemetry register offsets within an 8-byte block
  enum LSS_ScriptTelemetryOffset : uint8_t {
    OFFSET_SCRIPT_STATE              = 0x00, // Script engine state
    OFFSET_SCRIPT_ID_LSB             = 0x01, // Script ID LSB
    OFFSET_SCRIPT_ID_MSB             = 0x02, // Script ID MSB
    OFFSET_SCRIPT_CURRENT_LINE_LSB   = 0x03, // Current line LSB
    OFFSET_SCRIPT_CURRENT_LINE_MSB   = 0x04, // Current line MSB
    OFFSET_SCRIPT_LAST_FINISHED_LSB  = 0x05, // Last finished line LSB
    OFFSET_SCRIPT_LAST_FINISHED_MSB  = 0x06  // Last finished line MSB
  };

  // Register Values/Commands
  enum LSS_GlobalCmdVal : uint8_t {
    GLOBAL_CMD_UNMUTE   = 0x00,
    GLOBAL_CMD_RESET    = 0x01,
    GLOBAL_CMD_MUTE_ALL = 0x02
  };

  enum LSS_PlayCtrlVal : uint8_t {
    PLAY_CTRL_STOP      = 0x00,
    PLAY_CTRL_PLAY      = 0x01,
    PLAY_CTRL_LOOP      = 0x03,
    PLAY_CTRL_PAUSE     = 0x05
  };

  enum LSS_LoadCmdVal : uint8_t {
    LOAD_CMD_LOAD       = 0x01,
    LOAD_CMD_FLUSH      = 0x02
  };

  enum LSS_ScriptCmdVal : uint8_t {
    SCRIPT_CMD_START    = 0x01,
    SCRIPT_CMD_STOP     = 0x02
  };

  uint8_t _activeSlot;

  IO_LSS(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress) : DFPlayerBase(firstVpin, nPins) {
    _I2CAddress = i2cAddress;
    _activeSlot = 0xFF;
    addDevice(this);
  }

  void _begin() override {
    I2CManager.begin();
    if (!I2CManager.exists(_I2CAddress)) {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("LSS error: Device not found at I2C address %s"), _I2CAddress.toString());
      return;
    }

    uint8_t sysStatusReg = REG_SYS_STATUS;
    uint8_t sysStatusVal = 0;
    if (I2CManager.read(_I2CAddress, &sysStatusVal, 1, &sysStatusReg, 1) == I2C_STATUS_OK) {
      DIAG(F("LSS found at address %s, SYS_STATUS: 0x%02X"), _I2CAddress.toString(), sysStatusVal);
      _deviceState = DEVSTATE_NORMAL;
    } else {
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("LSS error: Failed to read SYS_STATUS from address %s"), _I2CAddress.toString());
    }
  }

  void _display() override {
    DIAG(F("LSS on I2C:%s Vpins:%u-%u %S"),
         _I2CAddress.toString(), (int)_firstVpin, (int)_firstVpin + _nPins - 1,
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }

  void _loop(unsigned long currentMicros) override {
    (void)currentMicros;
    // No background task processing or delay required for EX_lss, 
    // as it communicates directly with the sound board via register writes.
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState != DEVSTATE_NORMAL) return;
    uint8_t offset = (uint8_t)(vpin - _firstVpin);

    // Audio Slots 0 - 15: Play / Stop
    if (offset <= OFFSET_PIN_SLOT_MAX) {
      if (_activeSlot != offset) {
        writeReg(REG_SLOT_SELECTOR, offset);
        _activeSlot = offset;
      }
      if (value) {
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PLAY); // PLAY
      } else {
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_STOP); // STOP
      }
    }
    // Script Engines 0 - 3: value=1 starts (if loaded), value=0 stops engine
    else if (offset >= OFFSET_PIN_SCRIPT_ENG_BASE && offset <= OFFSET_PIN_SCRIPT_ENG_MAX) {
      uint8_t engine = offset - OFFSET_PIN_SCRIPT_ENG_BASE;
      writeReg(REG_SCRIPT_ENGINE, engine);
      writeReg(REG_SCRIPT_CMD, value ? SCRIPT_CMD_START : SCRIPT_CMD_STOP);
    }
  }

  int _read(VPIN vpin) override {
    if (_deviceState != DEVSTATE_NORMAL) return 0;
    uint8_t offset = (uint8_t)(vpin - _firstVpin);

    // Audio Slots 0 - 15: Returns 1 if playing, 0 if idle (WAITFOR support)
    if (offset <= OFFSET_PIN_SLOT_MAX) {
      if (_activeSlot != offset) {
        writeReg(REG_SLOT_SELECTOR, offset);
        _activeSlot = offset;
      }
      uint8_t playStatusVal = 0;
      if (readReg(REG_PLAY_STATUS, playStatusVal) == I2C_STATUS_OK) {
        return (playStatusVal & 0x01) ? 1 : 0; // Return 1 if IS_PLAYING (bit 0) is active
      }
      return 0;
    }

    // Script Engines 0 - 3 State: Returns 1 if busy/running, 0 if IDLE (WAITFOR support)
    if (offset >= OFFSET_PIN_SCRIPT_ENG_BASE && offset <= OFFSET_PIN_SCRIPT_ENG_MAX) {
      uint8_t engine = offset - OFFSET_PIN_SCRIPT_ENG_BASE;
      uint8_t reg = REG_SCRIPT_ENG0_BASE + (engine * 8); // OFFSET_SCRIPT_STATE is 0x00
      uint8_t stateVal = 0;
      if (readReg(reg, stateVal) == I2C_STATUS_OK) {
        return (stateVal != 0) ? 1 : 0; // 1 = running/busy, 0 = idle
      }
      return 0;
    }

    // Global System Status: Returns 1 if audio engine ready (bit 4)
    if (offset == OFFSET_PIN_SYS_STATUS) {
      uint8_t sysStatusVal = 0;
      if (readReg(REG_SYS_STATUS, sysStatusVal) == I2C_STATUS_OK) {
        return (sysStatusVal & 0x10) ? 1 : 0; // Bit 4 = AUDIO_ENGINE_READY
      }
      return 0;
    }

    return 0;
  }

  int _readAnalogue(VPIN vpin) override {
    if (_deviceState != DEVSTATE_NORMAL) return 0;
    uint8_t offset = (uint8_t)(vpin - _firstVpin);

    // Audio Slots 0 - 15: Returns REG_SLOT_STATUS (0x23: Empty, Busy, Ready, Error)
    if (offset <= OFFSET_PIN_SLOT_MAX) {
      if (_activeSlot != offset) {
        writeReg(REG_SLOT_SELECTOR, offset);
        _activeSlot = offset;
      }
      uint8_t slotStatusVal = 0;
      if (readReg(REG_SLOT_STATUS, slotStatusVal) == I2C_STATUS_OK) {
        return slotStatusVal;
      }
      return 0;
    }

    // Script Engines 0 - 3 State: Returns raw state byte (0x00 IDLE, 0x01 EXEC, etc.)
    if (offset >= OFFSET_PIN_SCRIPT_ENG_BASE && offset <= OFFSET_PIN_SCRIPT_ENG_MAX) {
      uint8_t engine = offset - OFFSET_PIN_SCRIPT_ENG_BASE;
      uint8_t reg = REG_SCRIPT_ENG0_BASE + (engine * 8);
      uint8_t stateVal = 0;
      if (readReg(reg, stateVal) == I2C_STATUS_OK) {
        return stateVal;
      }
      return 0;
    }

    // Script Engines 0 - 3 Current Line: Returns 16-bit line number
    if (offset >= OFFSET_PIN_SCRIPT_LINE_BASE && offset <= OFFSET_PIN_SCRIPT_LINE_MAX) {
      uint8_t engine = offset - OFFSET_PIN_SCRIPT_LINE_BASE;
      uint8_t reg = REG_SCRIPT_ENG0_BASE + (engine * 8) + OFFSET_SCRIPT_CURRENT_LINE_LSB;
      uint16_t lineVal = 0;
      if (readReg16(reg, lineVal) == I2C_STATUS_OK) {
        return (int)lineVal;
      }
      return 0;
    }

    // Global Hardware System Status: Returns 0x02 bitmask
    if (offset == OFFSET_PIN_SYS_STATUS) {
      uint8_t sysStatusVal = 0;
      if (readReg(REG_SYS_STATUS, sysStatusVal) == I2C_STATUS_OK) {
        return sysStatusVal;
      }
      return 0;
    }

    // Internal Die Temperature: Signed 16-bit integer, tenths of deg C (e.g. 384 = 38.4 C)
    if (offset == OFFSET_PIN_TEMP) {
      uint16_t rawTemp = 0;
      if (readReg16(REG_TEMP_LSB, rawTemp) == I2C_STATUS_OK) {
        return (int16_t)rawTemp;
      }
      return 0;
    }

    // INA219 Current: Unsigned 16-bit integer, Milliamps (mA)
    if (offset == OFFSET_PIN_CURRENT) {
      uint16_t currentVal = 0;
      if (readReg16(REG_CURRENT_LSB, currentVal) == I2C_STATUS_OK) {
        return (int)currentVal;
      }
      return 0;
    }

    // INA219 Voltage: Unsigned 16-bit integer, Millivolts (mV)
    if (offset == OFFSET_PIN_VOLTAGE) {
      uint16_t voltVal = 0;
      if (readReg16(REG_VOLTAGE_LSB, voltVal) == I2C_STATUS_OK) {
        return (int)voltVal;
      }
      return 0;
    }

    // INA219 Power: Unsigned 16-bit integer, Milliwatts (mW)
    if (offset == OFFSET_PIN_POWER) {
      uint16_t powerVal = 0;
      if (readReg16(REG_POWER_LSB, powerVal) == I2C_STATUS_OK) {
        return (int)powerVal;
      }
      return 0;
    }

    // Core 0 Peak Load (0 - 100%)
    if (offset == OFFSET_PIN_CORE0_LOAD) {
      uint8_t loadVal = 0;
      if (readReg(REG_CORE0_PEAK_LOAD, loadVal) == I2C_STATUS_OK) {
        return loadVal;
      }
      return 0;
    }

    // Core 1 Peak Load (0 - 100%)
    if (offset == OFFSET_PIN_CORE1_LOAD) {
      uint8_t loadVal = 0;
      if (readReg(REG_CORE1_PEAK_LOAD, loadVal) == I2C_STATUS_OK) {
        return loadVal;
      }
      return 0;
    }

    // OLED Page (0 - 5)
    if (offset == OFFSET_PIN_OLED_PAGE) {
      uint8_t pageVal = 0;
      if (readReg(REG_OLED_PAGE, pageVal) == I2C_STATUS_OK) {
        return pageVal;
      }
      return 0;
    }

    return 0;
  }

  void _writeAnalogue(VPIN vpin, int v1, uint8_t v2=0, uint16_t cmd=0) override {
    if (_deviceState != DEVSTATE_NORMAL) return;

    uint8_t offset = (uint8_t)(vpin - _firstVpin);
    if (offset == OFFSET_PIN_OLED_PAGE) {
      writeReg(REG_OLED_PAGE, (uint8_t)v1);
      return;
    }

    auto selectSlot = [this, vpin]() {
      uint8_t slot = (uint8_t)(vpin - _firstVpin);
      if (slot <= OFFSET_PIN_SLOT_MAX && _activeSlot != slot) {
        writeReg(REG_SLOT_SELECTOR, slot);
        _activeSlot = slot;
      }
    };

    switch (cmd & 0xFF) {
      // Standard DFPlayer compatibility mapping
      case DF_PLAY:
      case DF_REPEATPLAY:
        selectSlot();
        writeReg16(REG_FILE_ID_LSB, (uint16_t)v1); // File ID LSB & MSB
        writeReg(REG_SLOT_COMMAND, LOAD_CMD_LOAD);           // LOAD
        writeReg(REG_PLAY_CONTROL, (cmd == DF_REPEATPLAY) ? PLAY_CTRL_LOOP : PLAY_CTRL_PLAY); // PLAY | LOOP or PLAY
        break;

      case DF_STOPPLAY:
        selectSlot();
        writeReg(REG_SLOT_COMMAND, LOAD_CMD_FLUSH);           // FLUSH
        break;

      case DF_PAUSE:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PAUSE);           // PLAY | PAUSE
        break;

      case DF_RESUME:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PLAY);           // PLAY
        break;

      case DF_VOL:
        selectSlot();
        // Write to channel 0 by default for standard single-channel DFPlayer compatibility
        writeReg(REG_VOL_OUT_BASE, (uint8_t)v2);
        break;

      case DF_RESET:
        writeReg(REG_GLOBAL_CMD, GLOBAL_CMD_RESET);           // GLOBAL_CMD RESET
        break;

      // LSS Specific Commands
      case DF_LSS_LOAD:
        selectSlot();
        writeReg16(REG_FILE_ID_LSB, (uint16_t)v1); // File ID LSB & MSB
        writeReg(REG_SLOT_COMMAND, LOAD_CMD_LOAD);           // LOAD
        break;

      case DF_LSS_FLUSH:
        selectSlot();
        writeReg(REG_SLOT_COMMAND, LOAD_CMD_FLUSH);           // FLUSH
        break;

      case DF_LSS_PLAY:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PLAY);           // PLAY
        break;

      case DF_LSS_PLAY_LOOP:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_LOOP);           // PLAY & LOOP
        break;

      case DF_LSS_STOP:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_STOP);           // STOP
        break;

      case DF_LSS_PAUSE:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PAUSE);           // PLAY | PAUSE
        break;

      case DF_LSS_RESUME:
        selectSlot();
        writeReg(REG_PLAY_CONTROL, PLAY_CTRL_PLAY);           // PLAY
        break;

      case DF_LSS_VOLUME:
        selectSlot();
        // v1 = volume, v2 = channel
        writeReg(REG_VOL_OUT_BASE + v2, (uint8_t)v1);
        break;

      case DF_LSS_FADE: {
        selectSlot();
        uint8_t curve = (uint8_t)((cmd >> 8) & 0x0F);
        uint8_t channel = (uint8_t)((cmd >> 12) & 0x0F);
        writeReg(REG_FADE_CHANNEL, channel);
        writeReg(REG_FADE_TARGET_VOL, (uint8_t)v2); // target_vol
        writeReg16(REG_FADE_DUR_LSB, (uint16_t)v1); // duration_ticks
        writeReg(REG_FADE_TRIGGER, curve); // trigger curve
        break;
      }

      case DF_LSS_GLOBAL_RESET:
        writeReg(REG_GLOBAL_CMD, GLOBAL_CMD_RESET);           // RESET
        break;

      case DF_LSS_GLOBAL_MUTE:
        writeReg(REG_GLOBAL_CMD, v1 ? GLOBAL_CMD_MUTE_ALL : GLOBAL_CMD_UNMUTE); // MUTE_ALL or UNMUTE
        break;

      case DF_LSS_OLED_PAGE:
        writeReg(REG_OLED_PAGE, (uint8_t)v1);
        break;

      case DF_LSS_RUN_SCRIPT: {
        uint8_t offset = (uint8_t)(vpin - _firstVpin);
        uint8_t engine = (offset >= OFFSET_PIN_SCRIPT_ENG_BASE && offset <= OFFSET_PIN_SCRIPT_ENG_MAX)
                         ? (offset - OFFSET_PIN_SCRIPT_ENG_BASE)
                         : (uint8_t)v2;
        writeReg(REG_SCRIPT_ENGINE, engine);
        writeReg16(REG_SCRIPT_ID, (uint16_t)v1); // script_id LSB/MSB
        writeReg(REG_SCRIPT_CMD, SCRIPT_CMD_START);           // START script
        break;
      }

      case DF_LSS_STOP_SCRIPT: {
        uint8_t offset = (uint8_t)(vpin - _firstVpin);
        uint8_t engine = (offset >= OFFSET_PIN_SCRIPT_ENG_BASE && offset <= OFFSET_PIN_SCRIPT_ENG_MAX)
                         ? (offset - OFFSET_PIN_SCRIPT_ENG_BASE)
                         : (uint8_t)v2;
        writeReg(REG_SCRIPT_ENGINE, engine);
        writeReg(REG_SCRIPT_CMD, SCRIPT_CMD_STOP);           // STOP script
        break;
      }
    }
  }

  // Helper functions for I2C register reads and writes
  uint8_t readReg(uint8_t reg, uint8_t &val) {
    uint8_t regAddr = reg;
    uint8_t status = I2CManager.read(_I2CAddress, &val, 1, &regAddr, 1);
#ifdef DEBUG_LSS_I2C
    DIAG(F("LSS I2C readReg: reg=0x%02X, val=0x%02X, status=%d"), reg, val, status);
#endif
    return status;
  }

  uint8_t readReg16(uint8_t reg, uint16_t &val) {
    uint8_t buf[2] = {0, 0};
    uint8_t regAddr = reg;
    uint8_t status = I2CManager.read(_I2CAddress, buf, 2, &regAddr, 1);
    if (status == I2C_STATUS_OK) {
      val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
#ifdef DEBUG_LSS_I2C
    DIAG(F("LSS I2C readReg16: reg=0x%02X, val=0x%04X, status=%d"), reg, val, status);
#endif
    return status;
  }

  uint8_t writeReg(uint8_t reg, uint8_t val) {
#ifdef DEBUG_LSS_I2C
    DIAG(F("LSS I2C write: reg=0x%02X, val=0x%02X"), reg, val);
#endif
    uint8_t data[] = {reg, val};
    return I2CManager.write(_I2CAddress, data, 2);
  }

  uint8_t writeReg16(uint8_t reg, uint16_t val) {
#ifdef DEBUG_LSS_I2C
    DIAG(F("LSS I2C write16: reg=0x%02X, val=0x%04X"), reg, val);
#endif
    uint8_t data[] = {reg, (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF)};
    return I2CManager.write(_I2CAddress, data, 3);
  }

  // Pure virtual requirements for DFPlayerBase
  void transmitCommandBuffer(const uint8_t buffer[], size_t bytes) override {
    (void)buffer; (void)bytes;
  }

  bool processIncoming() override {
    return false;
  }
};

#endif
