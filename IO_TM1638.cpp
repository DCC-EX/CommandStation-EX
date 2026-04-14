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

/* Credit to https://github.com/dvarrel/TM1638 for the basic formulae.*/


#include <Arduino.h>
#include "IODevice.h"
#include "IO_TM1638.h"
#include "DIAG.h"

   
const uint8_t HIGHFLASH _digits[16]={
      0b00111111,0b00000110,0b01011011,0b01001111,
      0b01100110,0b01101101,0b01111101,0b00000111,
      0b01111111,0b01101111,0b01110111,0b01111100,
      0b00111001,0b01011110,0b01111001,0b01110001
    };

  // Constructor
   TM1638::TM1638(VPIN firstVpin, byte clk_pin,byte dio_pin,byte stb_pin){
    _firstVpin = firstVpin;
    _nPins = 8;
    _clk_pin = clk_pin;
    _stb_pin = stb_pin;
    _dio_pin = dio_pin;
    pinMode(clk_pin,OUTPUT);
    pinMode(stb_pin,OUTPUT);
    pinMode(dio_pin,OUTPUT);
    _pulse = PULSE1_16;
      
    _buttons=0;
    _leds=0;
    _lastLoop=micros();
    addDevice(this);
   } 

    
  void TM1638::create(VPIN firstVpin, byte clk_pin,byte dio_pin,byte stb_pin) {
    if (checkNoOverlap(firstVpin,8)) 
     new TM1638(firstVpin, clk_pin,dio_pin,stb_pin); 
  }

  void TM1638::_begin()  {
    displayClear();
    test();
    _display();
  }
  
 
  void TM1638::_loop(unsigned long currentMicros)  {
     if (currentMicros - _lastLoop > (1000000UL/LoopHz)) {
         _buttons=getButtons();// Read the buttons
         _lastLoop=currentMicros;   
     } 
  }
           
  void TM1638::_display()  {
    DIAG(F("TM1638 Configured on Vpins:%u-%u"), _firstVpin, _firstVpin+_nPins-1);
  }

// digital read gets button state 
int TM1638::_read(VPIN vpin)  {
  byte pin=vpin - _firstVpin;
  bool result=bitRead(_buttons,pin);
  // DIAG(F("TM1638 read (%d) buttons %x = %d"),pin,_buttons,result);  
  return result;
}

// digital write sets led state 
void TM1638::_write(VPIN vpin, int value)  {
  // TODO.. skip if no state change  
  writeLed(vpin - _firstVpin + 1,value!=0);
  }

// Analog write sets digit displays 

void TM1638::_writeAnalogue(VPIN vpin, int lowBytes, uint8_t mode, uint16_t highBytes)  {  
   // mode is in DataFormat defined above.
   byte formatLength=mode & 0x0F;  // last 4 bits 
   byte formatType=mode & 0xF0;         //           
   int8_t leftDigit=vpin-_firstVpin; // 0..7 from left
   int8_t rightDigit=leftDigit+formatLength-1; // 0..7 from left
   
   // loading is done right to left startDigit first
   int8_t startDigit=7-rightDigit; // reverse as 7 on left
   int8_t lastDigit=7-leftDigit; // reverse as 7 on left
   uint32_t value=highBytes;
   value<<=16;
   value |= (uint16_t)lowBytes;
   
   //DIAG(F("TM1638 fl=%d ft=%x sd=%d ld=%d v=%l vx=%X"),
   // formatLength,formatType,startDigit,lastDigit,value,value);
    while(startDigit<=lastDigit) {
        switch (formatType) {
            case _DF_DECIMAL:// decimal (leading zeros)
                displayDig(startDigit,GETHIGHFLASH(_digits,(value%10))); 
                value=value/10;
                break; 
            case _DF_HEX:// HEX (leading zeros)
                displayDig(startDigit,GETHIGHFLASH(_digits,(value & 0x0F))); 
                value>>=4;
                break;  
            case _DF_RAW:// Raw 7-segment pattern 
                displayDig(startDigit,value & 0xFF); 
                value>>=8;
                break;
            default:
                DIAG(F("TM1368 invalid mode 0x%x"),mode);
                return;
            }
    startDigit++;
    } 
}
     
uint8_t TM1638::getButtons(){
  ArduinoPins::fastWriteDigital(_stb_pin, LOW);
  writeData(INSTRUCTION_READ_KEY);
  pinMode(_dio_pin, INPUT);
  ArduinoPins::fastWriteDigital(_clk_pin, LOW);
  uint8_t buttons=0;
  for (uint8_t eachByte=0; eachByte<4;eachByte++) {
    uint8_t value = 0;
	  for (uint8_t eachBit = 0; eachBit < 8; eachBit++) {
		  ArduinoPins::fastWriteDigital(_clk_pin, HIGH);
			value |= ArduinoPins::fastReadDigital(_dio_pin) << eachBit;
		  ArduinoPins::fastWriteDigital(_clk_pin, LOW);
	  }
    buttons |= value << eachByte; 
    delayMicroseconds(1);
  }
  pinMode(_dio_pin, OUTPUT);
  ArduinoPins::fastWriteDigital(_stb_pin, HIGH);
  return buttons;
}


void TM1638::displayDig(uint8_t digitId, uint8_t pgfedcba){
  if (digitId>7) return;
  setDataInstruction(DISPLAY_TURN_ON | _pulse);
  setDataInstruction(INSTRUCTION_WRITE_DATA| INSTRUCTION_ADDRESS_FIXED);
  writeDataAt(FIRST_DISPLAY_ADDRESS+14-(digitId*2), pgfedcba);
}

void TM1638::displayClear(){
  setDataInstruction(DISPLAY_TURN_ON | _pulse);
  setDataInstruction(INSTRUCTION_WRITE_DATA | INSTRUCTION_ADDRESS_FIXED);
  for (uint8_t i=0;i<15;i+=2){
    writeDataAt(FIRST_DISPLAY_ADDRESS+i,0x00);
  }
}

void TM1638::writeLed(uint8_t num,bool state){
  if ((num<1) | (num>8)) return;
  setDataInstruction(DISPLAY_TURN_ON | _pulse);
  setDataInstruction(INSTRUCTION_WRITE_DATA | INSTRUCTION_ADDRESS_FIXED);
  writeDataAt(FIRST_DISPLAY_ADDRESS + (num*2-1), state);
}


void TM1638::writeData(uint8_t data){
	for (uint8_t i = 0; i < 8; i++)  {
		ArduinoPins::fastWriteDigital(_dio_pin, data & 1);
		data >>= 1;
		ArduinoPins::fastWriteDigital(_clk_pin, HIGH);
		ArduinoPins::fastWriteDigital(_clk_pin, LOW);		
	}
} 

void TM1638::writeDataAt(uint8_t displayAddress, uint8_t data){
    ArduinoPins::fastWriteDigital(_stb_pin, LOW);
    writeData(displayAddress);
    writeData(data);
    ArduinoPins::fastWriteDigital(_stb_pin, HIGH);
    delayMicroseconds(1);
}

void TM1638::setDataInstruction(uint8_t dataInstruction){
  ArduinoPins::fastWriteDigital(_stb_pin, LOW);
  writeData(dataInstruction);
  ArduinoPins::fastWriteDigital(_stb_pin, HIGH);
  delayMicroseconds(1);  
}

void TM1638::test(){
  DIAG(F("TM1638 test"));
  uint8_t val=0;
  for(uint8_t i=0;i<5;i++){
    setDataInstruction(DISPLAY_TURN_ON | _pulse);
    setDataInstruction(INSTRUCTION_WRITE_DATA| INSTRUCTION_ADDRESS_AUTO);
    ArduinoPins::fastWriteDigital(_stb_pin, LOW);
    writeData(FIRST_DISPLAY_ADDRESS);
    for(uint8_t i=0;i<16;i++)
      writeData(val);
    ArduinoPins::fastWriteDigital(_stb_pin, HIGH);
    delay(1000);
    val = ~val;
  }

}
