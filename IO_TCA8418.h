/*
 *  © 2023-2024, Paul M. Antoine
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

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for TCA8418 80-key keypad encoder, which we'll treat as 80 available VPINs where
 * key down == 1 and key up == 0 by configuring just as an 8x10 keyboard matrix. Users can opt to use
 * up to all 80 of the available VPINs for now, allowing memory to be saved if not all events are required.
 * 
 * The datasheet says:
 * 
 * The TCA8418 can be configured to support many different configurations of keypad setups.
 * All 18 GPIOs for the rows and columns can be used to support up to 80 keys in an 8x10 key pad
 * array. Another option is that all 18 GPIOs be used for GPIs to read 18 buttons which are
 * not connected in an array. Any combination in between is also acceptable (for example, a
 * 3x4 keypad matrix and using the remaining 11 GPIOs as a combination of inputs and outputs).
 * 
 * With an 8x10 key event matrix, the events are numbered as such:
 * 
 *     C0  C1  C2  C3  C4  C5  C6  C7  C8  C9
 *    ========================================
 * R0|  0   1   2   3   4   5   6   7   8   9
 * R1| 10  11  12  13  14  15  16  17  18  19
 * R2| 20  21  22  23  24  25  26  27  28  29
 * R3| 30  31  32  33  34  35  36  37  38  39
 * R4| 40  41  42  43  44  45  46  47  48  49
 * R5| 50  51  52  53  54  55  56  57  58  59
 * R6| 60  61  62  63  64  65  66  67  68  69
 * R7| 70  71  72  73  74  75  76  77  78  79
 * 
 * So if you start with VPIN 300, R0/C0 will be 300, and R7/C9 will be 379.
 * 
 * HAL declaration for myAutomation.h is:
 * HAL(TCA8418, firstVpin, numPins, I2CAddress, interruptPin)
 * 
 * Where numPins can be 1-80, and interruptPin can be any spare Arduino pin.
 * 
 * Configure using the following on the main I2C bus:
 * HAL(TCA8418, 300, 80, 0x34)
 * 
 * Use something like this on a multiplexor, and with up to 8 of the 8-way multiplexors you could have 64 different TCA8418 boards:
 * HAL(TCA8418, 400, 80, {SubBus_1, 0x34})
 * 
 * And if needing an Interrupt pin to speed up operations:
 * HAL(TCA8418, 300, 80, 0x34, D21)
 * 
 * Note that using an interrupt pin speeds up button press acquisition considerably (less than a millisecond vs 10-100),
 * but even with interrupts enabled the code presently checks every 100ms in case the interrupt pin becomes disconnected.
 * Use any available Arduino pin for interrupt monitoring.
 */
 
class TCA8418 : public IODevice {
public:

  static void create(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress))
      new TCA8418(firstVpin, (nPins = (nPins > 80) ? 80 : nPins), i2cAddress, interruptPin);
  }

private:  

  uint8_t* _digitalInputStates = NULL;  // Array of pin states
  uint8_t _digitalPinBytes = 0;         // Number of bytes in pin state array

  uint8_t _numKeyEvents = 0;            // Number of outsanding key events waiting for us

  unsigned long _lastEventRead = 0;
  unsigned long _eventRefresh = 10000UL;    // Delay refreshing events for 10ms
  const unsigned long _eventRefreshSlow = 100000UL;   // Delay refreshing events for 100ms
  bool _gpioInterruptsEnabled = false;

  uint8_t _inputBuffer[1];
  uint8_t _commandBuffer[1];
  I2CRB _i2crb;

  enum {RDS_IDLE, RDS_EVENT, RDS_KEYCODE};  // Read operation states
  uint8_t _readState = RDS_IDLE;

  // Constructor
  TCA8418(VPIN firstVpin, uint8_t nPins, I2CAddress i2cAddress, int interruptPin=-1) {
    if (nPins > 0)
    {
      _firstVpin = firstVpin;
      _nPins = nPins;
      _I2CAddress = i2cAddress;
      _gpioInterruptPin = interruptPin;
      addDevice(this);
    }
  }

  void _begin() {

    I2CManager.begin();

    if (I2CManager.exists(_I2CAddress)) {
      // Default all GPIO pins to INPUT
      I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR_1, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR_2, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_DIR_3, 0x00);

      // Remove all GPIO pins from events
      I2CManager.write(_I2CAddress, 2, REG_GPI_EM_1, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPI_EM_2, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPI_EM_3, 0x00);

      // Set all pins to FALLING interrupts
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_LVL_1, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_LVL_2, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_LVL_3, 0x00);

      // Remove all GPIO pins from interrupts
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN_1, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN_2, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_GPIO_INT_EN_3, 0x00);

      // Set up an 8 x 10 matrix by writing 0xFF to all the row and column configs
      // Row config is maximum of 8, and in REG_KP_GPIO_1
      I2CManager.write(_I2CAddress, 2, REG_KP_GPIO_1, 0xFF);
      // Column config is maximum of 10, lower 8 bits in REG_KP_GPIO_2, upper in REG_KP_GPIO_3
      // Set first 8 columns
      I2CManager.write(_I2CAddress, 2, REG_KP_GPIO_2, 0xFF);
      // Turn on cols 9/10
      I2CManager.write(_I2CAddress, 2, REG_KP_GPIO_3, 0x03);

      // // Set all pins to Enable Debounce
      I2CManager.write(_I2CAddress, 2, REG_DEBOUNCE_DIS_1, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_DEBOUNCE_DIS_2, 0x00);
      I2CManager.write(_I2CAddress, 2, REG_DEBOUNCE_DIS_3, 0x00);

      // Let's assume an 8x10 matrix for now, and configure 
      _digitalPinBytes = (_nPins + 7) / 8;
      if ((_digitalInputStates = (byte *)calloc(_digitalPinBytes, 1)) == NULL) {
        DIAG(F("TCA8418 I2C: Unable to alloc %d bytes"), _digitalPinBytes);
        return;
      }

    // Configure pin used for GPIO extender notification of change (if allocated)
    // and configure TCA8418 to produce key event interrupts
    if (_gpioInterruptPin >= 0) {
      DIAG(F("TCA8418 I2C: interrupt pin configured on %d"), _gpioInterruptPin);
      _gpioInterruptsEnabled = true;
      _eventRefresh = _eventRefreshSlow; // Switch to slower manual refreshes in case the INT pin isn't connected!
      pinMode(_gpioInterruptPin, INPUT_PULLUP);
      I2CManager.write(_I2CAddress, 2, REG_CFG, REG_CFG_KE_IEN);
      // Clear any pending interrupts
      I2CManager.write(_I2CAddress, 2, REG_INT_STAT, REG_STAT_K_INT);
    }

#ifdef DIAG_IO
      _display();
#endif
    }
  }

  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED)
      return 0;
    int pin = vpin - _firstVpin;
    bool result = _digitalInputStates[pin / 8] & (1 << (pin % 8));
    return result;
  }


  // Main loop, collect both digital and analogue pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;    // If device failed, return

    // Request block is used for key event reads from the TCA8418, which are performed
    // on a cyclic basis.

    if (_readState != RDS_IDLE) {
      if (_i2crb.isBusy()) return;                // If I2C operation still in progress, return

      uint8_t status = _i2crb.status;
      if (status == I2C_STATUS_OK) {             // If device request ok, read input data

        // First check if we have any key events waiting
        if (_readState == RDS_EVENT) {
          if ((_numKeyEvents = (_inputBuffer[0] & 0x0F)) != 0) {
            // We could read each key event waiting in a synchronous loop, which may prove preferable
            // but for now, schedule an async read of the first key event in the queue
            _commandBuffer[0] = REG_KEY_EVENT_A;
            I2CManager.read(_I2CAddress, _inputBuffer, 1, _commandBuffer, 1, &_i2crb);  // non-blocking read
            _readState = RDS_KEYCODE; // Shift to reading key events!
          }
          else // We found no key events waiting, return to IDLE
            _readState = RDS_IDLE;
        }
         else {
          // RDS_KEYCODE
          uint8_t key = _inputBuffer[0] & 0x7F;
          bool keyDown = _inputBuffer[0] & 0x80;
          // Check for just keypad events
          key--; // R0/C0 is key #1, so subtract 1 to create an array offset
          // We only want to record key events we're configured for, as we have calloc'd an
          // appropriately sized _digitalInputStates array!
          if (key < _nPins) {
            if (keyDown)
              _digitalInputStates[key / 8] |= (1 << (key % 8));
            else
              _digitalInputStates[key / 8] &= ~(1 << (key % 8));
          }
          else
            DIAG(F("TCA8418 I2C: key event %d discarded, outside Vpin range"), key);
          _numKeyEvents--; // One less key event to get
          if (_numKeyEvents != 0)
          {
            // DIAG(F("TCA8418 I2C: more keys in read event queue, # waiting is: %x"), _numKeyEvents);
            // We could read each key event waiting in a synchronous loop, which may prove preferable
            // but for now, schedule an async read of the first key event in the queue
            _commandBuffer[0] = REG_KEY_EVENT_A;
            I2CManager.read(_I2CAddress, _inputBuffer, 1, _commandBuffer, 1, &_i2crb); // non-blocking read
          }
          else {
            // DIAG(F("TCA8418 I2C: no more keys in read event queue"));
            // Clear any pending interrupts
            I2CManager.write(_I2CAddress, 2, REG_INT_STAT, REG_STAT_K_INT);
            _readState = RDS_IDLE; // Shift to IDLE
            return;
          }
         }
      } else
        reportError(status, false);   // report eror but don't go offline.
    }

    // If we're not doing anything now, check to see if we have an interrupt pin configured and it is low,
    // or if our timer has elapsed and we should check anyway in case the interrupt pin is disconnected.
    if (_readState == RDS_IDLE) {
      if ((_gpioInterruptsEnabled && !digitalRead(_gpioInterruptPin)) ||
        ((currentMicros - _lastEventRead) > _eventRefresh))
      {
        _commandBuffer[0] = REG_KEY_LCK_EC;
        I2CManager.read(_I2CAddress, _inputBuffer, 1, _commandBuffer, 1, &_i2crb);  // non-blocking read
        _lastEventRead = currentMicros;
        _readState = RDS_EVENT; // Shift to looking for key events!
      }
    }
  }

  // Display device information and status
  void _display() override {
    DIAG(F("TCA8418 I2C:%s Vpins %u-%u%S"),
              _I2CAddress.toString(),
              _firstVpin, (_firstVpin+_nPins-1),
              _deviceState == DEVSTATE_FAILED ? F(" OFFLINE") : F(""));
    if (_gpioInterruptsEnabled)
      DIAG(F("TCA8418 I2C:Interrupt on pin %d"), _gpioInterruptPin);
  }

  // Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("TCA8418 I2C:%s Error:%d (%S)"), _I2CAddress.toString(), 
      status, I2CManager.getErrorMessage(status));
    if (fail)
    _deviceState = DEVSTATE_FAILED;
  }

  enum tca8418_registers
  {
    // REG_RESERVED = 0x00
    REG_CFG = 0x01,             // Configuration register
    REG_INT_STAT = 0x02,        // Interrupt status
    REG_KEY_LCK_EC = 0x03,      // Key lock and event counter
    REG_KEY_EVENT_A = 0x04,     // Key event register A
    REG_KEY_EVENT_B = 0x05,     // Key event register B
    REG_KEY_EVENT_C = 0x06,     // Key event register C
    REG_KEY_EVENT_D = 0x07,     // Key event register D
    REG_KEY_EVENT_E = 0x08,     // Key event register E
    REG_KEY_EVENT_F = 0x09,     // Key event register F
    REG_KEY_EVENT_G = 0x0A,     // Key event register G
    REG_KEY_EVENT_H = 0x0B,     // Key event register H
    REG_KEY_EVENT_I = 0x0C,     // Key event register I
    REG_KEY_EVENT_J = 0x0D,     // Key event register J
    REG_KP_LCK_TIMER = 0x0E,    // Keypad lock1 to lock2 timer
    REG_UNLOCK_1 = 0x0F,        // Unlock register 1
    REG_UNLOCK_2 = 0x10,        // Unlock register 2
    REG_GPIO_INT_STAT_1 = 0x11, // GPIO interrupt status 1
    REG_GPIO_INT_STAT_2 = 0x12, // GPIO interrupt status 2
    REG_GPIO_INT_STAT_3 = 0x13, // GPIO interrupt status 3
    REG_GPIO_DAT_STAT_1 = 0x14, // GPIO data status 1
    REG_GPIO_DAT_STAT_2 = 0x15, // GPIO data status 2
    REG_GPIO_DAT_STAT_3 = 0x16, // GPIO data status 3
    REG_GPIO_DAT_OUT_1 = 0x17,  // GPIO data out 1
    REG_GPIO_DAT_OUT_2 = 0x18,  // GPIO data out 2
    REG_GPIO_DAT_OUT_3 = 0x19,  // GPIO data out 3
    REG_GPIO_INT_EN_1 = 0x1A,   // GPIO interrupt enable 1
    REG_GPIO_INT_EN_2 = 0x1B,   // GPIO interrupt enable 2
    REG_GPIO_INT_EN_3 = 0x1C,   // GPIO interrupt enable 3
    REG_KP_GPIO_1 = 0x1D,       // Keypad/GPIO select 1
    REG_KP_GPIO_2 = 0x1E,       // Keypad/GPIO select 2
    REG_KP_GPIO_3 = 0x1F,       // Keypad/GPIO select 3
    REG_GPI_EM_1 = 0x20,        // GPI event mode 1
    REG_GPI_EM_2 = 0x21,        // GPI event mode 2
    REG_GPI_EM_3 = 0x22,        // GPI event mode 3
    REG_GPIO_DIR_1 = 0x23,      // GPIO data direction 1
    REG_GPIO_DIR_2 = 0x24,      // GPIO data direction 2
    REG_GPIO_DIR_3 = 0x25,      // GPIO data direction 3
    REG_GPIO_INT_LVL_1 = 0x26,  // GPIO edge/level detect 1
    REG_GPIO_INT_LVL_2 = 0x27,  // GPIO edge/level detect 2
    REG_GPIO_INT_LVL_3 = 0x28,  // GPIO edge/level detect 3
    REG_DEBOUNCE_DIS_1 = 0x29,  // Debounce disable 1
    REG_DEBOUNCE_DIS_2 = 0x2A,  // Debounce disable 2
    REG_DEBOUNCE_DIS_3 = 0x2B,  // Debounce disable 3
    REG_GPIO_PULL_1 = 0x2C,     // GPIO pull-up disable 1
    REG_GPIO_PULL_2 = 0x2D,     // GPIO pull-up disable 2
    REG_GPIO_PULL_3 = 0x2E,     // GPIO pull-up disable 3
    // REG_RESERVED = 0x2F
  };

  enum tca8418_config_reg_fields
  {
    //  Config Register #1 fields
    REG_CFG_AI = 0x80,           // Auto-increment for read/write
    REG_CFG_GPI_E_CGF = 0x40,    // Event mode config
    REG_CFG_OVR_FLOW_M = 0x20,   // Overflow mode enable
    REG_CFG_INT_CFG = 0x10,      // Interrupt config
    REG_CFG_OVR_FLOW_IEN = 0x08, // Overflow interrupt enable
    REG_CFG_K_LCK_IEN = 0x04,    // Keypad lock interrupt enable
    REG_CFG_GPI_IEN = 0x02,      // GPI interrupt enable
    REG_CFG_KE_IEN = 0x01,       // Key events interrupt enable
  };

  enum tca8418_int_status_fields
  {
    //  Interrupt Status Register #2 fields
    REG_STAT_CAD_INT = 0x10,      // Ctrl-alt-del seq status
    REG_STAT_OVR_FLOW_INT = 0x08, // Overflow interrupt status
    REG_STAT_K_LCK_INT = 0x04,    // Key lock interrupt status
    REG_STAT_GPI_INT = 0x02,      // GPI interrupt status
    REG_STAT_K_INT = 0x01,        // Key events interrupt status
  };

  enum tca8418_lock_ec_fields
  {
    // Key Lock Event Count Register #3
    REG_LCK_EC_K_LCK_EN = 0x40, // Key lock enable
    REG_LCK_EC_LCK_2 = 0x20,    // Keypad lock status 2
    REG_LCK_EC_LCK_1 = 0x10,    // Keypad lock status 1
    REG_LCK_EC_KLEC_3 = 0x08,   // Key event count bit 3
    REG_LCK_EC_KLEC_2 = 0x04,   // Key event count bit 2
    REG_LCK_EC_KLEC_1 = 0x02,   // Key event count bit 1
    REG_LCK_EC_KLEC_0 = 0x01,   // Key event count bit 0
  };
};

#endif
