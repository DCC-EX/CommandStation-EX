   /*
 *  © 2024, Chris Harlow. All rights reserved.
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

#ifndef IO_TM1638_h
#define IO_TM1638_h
#include <Arduino.h>
#include "IODevice.h"
#include "DIAG.h"

class TM1638 : public IODevice {
private: 
      
    uint8_t _buttons;
    uint8_t _leds;
    unsigned long _lastLoop;
    static const int LoopHz=20; 

    static const byte 
    INSTRUCTION_WRITE_DATA=0x40,
    INSTRUCTION_READ_KEY=0x42,
    INSTRUCTION_ADDRESS_AUTO=0x40,
    INSTRUCTION_ADDRESS_FIXED=0x44,
    INSTRUCTION_NORMAL_MODE=0x40,
    INSTRUCTION_TEST_MODE=0x48,

    FIRST_DISPLAY_ADDRESS=0xC0,

    DISPLAY_TURN_OFF=0x80,
    DISPLAY_TURN_ON=0x88;

        
    uint8_t _clk_pin;
    uint8_t _stb_pin;
    uint8_t _dio_pin;
    uint8_t _pulse;
    bool _isOn;

    
  // Constructor
   TM1638(VPIN firstVpin, byte clk_pin,byte dio_pin,byte stb_pin);

public:
  enum DigitFormat : byte {
    // last 4 bits are length.
    // DF_1.. DF_8 decimal 
    DF_1=0x01,DF_2=0x02,DF_3=0x03,DF_4=0x04,
    DF_5=0x05,DF_6=0x06,DF_7=0x07,DF_8=0x08,
    // DF_1X.. DF_8X HEX 
    DF_1X=0x11,DF_2X=0x12,DF_3X=0x13,DF_4X=0x14,
    DF_5X=0x15,DF_6X=0x16,DF_7X=0x17,DF_8X=0x18,
    // DF_1R .. DF_4R raw 7 segmnent data 
    // only 4 because HAL analogWrite only passes 4 bytes 
    DF_1R=0x21,DF_2R=0x22,DF_3R=0x23,DF_4R=0x24,
    
    //  bits of data conversion type  (ored with length) 
    _DF_DECIMAL=0x00,// right adjusted decimal unsigned leading zeros
    _DF_HEX=0x10,    // right adjusted hex leading zeros 
    _DF_RAW=0x20 // bytes are raw 7-segment pattern (max length 4)
  };

  static void create(VPIN firstVpin, byte clk_pin,byte dio_pin,byte stb_pin);
  
  // Functions overridden in IODevice
  void _begin();
  void _loop(unsigned long currentMicros) override ;
  void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override;
  void _display() override ;
  int _read(VPIN pin) override;
  void _write(VPIN pin,int value) override;

  // Device driving functions 
  private:
   enum pulse_t {
      PULSE1_16,
      PULSE2_16,
      PULSE4_16,
      PULSE10_16,
      PULSE11_16,
      PULSE12_16,
      PULSE13_16,
      PULSE14_16
    };

  /**
    * @fn getButtons
    * @return state of 8 buttons
    */
    uint8_t getButtons();

    /**
    * @fn writeLed
    * @brief put led ON or OFF
    * @param num num of led(1-8)
    * @param state (true or false)
    */
    void writeLed(uint8_t num, bool state);
    
    
    /**
    * @fn displayDig
    * @brief set 7 segment display + dot
    * @param digitId num of digit(0-7)
    * @param val value 8 bits
    */
    void displayDig(uint8_t digitId, uint8_t pgfedcba);

    /**
    * @fn displayClear
    * @brief switch off all leds and segment display
    */
    void displayClear();
    void test();
    void writeData(uint8_t data);
    void writeDataAt(uint8_t displayAddress, uint8_t data);
    void setDisplayMode(uint8_t displayMode);
    void setDataInstruction(uint8_t dataInstruction);
};
#endif
