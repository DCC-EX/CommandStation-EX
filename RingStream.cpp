/*
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX CommandStation-EX
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


// NOTE: The use of a marker byte without an escape algorithm means
// RingStream is unsuitable for binary data. Should binary data need to be 
// streamed it will be necessary to implementr an escape strategy to handle the 
// marker char when embedded in data. 

#include "RingStream.h"
#include "DIAG.h"

const byte FLASH_INSERT_MARKER=0xff;

RingStream::RingStream( const uint16_t len)
{
  _len=len;
  _buffer=new byte[len];
  _pos_write=0;
  _pos_read=0;
  _buffer[0]=0;
  _overflow=false;
  _mark=0;
  _count=0; 
  _flashInsert=0;
}

size_t RingStream::write(uint8_t b) {
  if (_overflow) return 0;
  _buffer[_pos_write] = b;
  ++_pos_write;
  if (_pos_write==_len) _pos_write=0;
  if (_pos_write==_pos_read) {
    _overflow=true; 
    return 0;
  }
  _count++;
  return 1;
}

// Ideally, I would prefer to override the Print:print(_FlashStringHelper) function
// but the library authors omitted to make this virtual.
// Therefore we obveride the only other simple function that has no side effects
// in order that StringFormatter can recognise a RingStream and call its 
// printFlash() directly.  
int RingStream::availableForWrite()  { 
  return  THIS_IS_A_RINGSTREAM; 
  }

size_t RingStream::printFlash(const FSH * flashBuffer) {
  // This function does not work on a 32 bit processor where the runtime
  // sometimes misrepresents the pointer size in uintptr_t.
  // In any case its not really necessary in a 32 bit processor because
  // we have adequate ram. 
  if (sizeof(void*)>2) return print(flashBuffer);


// We are about to add a PROGMEM string to the buffer. 
// To save RAM we can insert a marker and the
// progmem address into the buffer instead.
// The buffer reading code must recognise this marker and 
// silently extract the progmem bytes.
// In addition, we must make the count correct as if the 
// string had been embedded so that things like the wifi code
// can read the expected count before reading the buffer. 

// Establish the actual length of the progmem string.
char * flash=(char *)flashBuffer;
int16_t plength=STRLEN_P(flash);
if (plength==0) return 0; // just ignore empty string

// Retain the buffer count as it will be modified by the marker+address insert
int prevCount=_count;
write(FLASH_INSERT_MARKER); // write the marker
uintptr_t iFlash=reinterpret_cast<uintptr_t>(flash); // expect size match with pointer

// write address bytes LSB first (size depends on CPU) 
for (byte f=0;f<sizeof(iFlash); f++) {
    write((byte) (iFlash & 0xFF));
    iFlash>>=8;
}

// correct the buffer count to reflect the flash length, not the marker/addr.
_count=prevCount+plength;  
return plength;
}

int RingStream::read() {
  if (_flashInsert) {
    // we are reading out of a flash string 
    byte fb=GETFLASH(_flashInsert);
    _flashInsert++;
    if (fb) return fb; // we have a byte from the flash
    // flash insert complete, clear and drop through to next buffer byte
    _flashInsert=NULL; 
  }
  if ((_pos_read==_pos_write) && !_overflow) return -1;  // empty  
  byte b=readRawByte();
  if (b!=FLASH_INSERT_MARKER) return b; 
  // Detected a flash insert 
  if (sizeof(void*)>2) {
    DIAG(F("Detected invalid flash insert marker at pos %d"),_pos_read);
    return '?';
  }
  // read address bytes LSB first (size depends on CPU) 
  uintptr_t iFlash=0; 
  for (byte f=0; f<sizeof(iFlash); f++) {
    uintptr_t bf=readRawByte();
    bf&=0x00ff;
    bf<<= (8*f); // shift byte to correct position in iFlash
    iFlash |= bf;  
  }
  _flashInsert=reinterpret_cast<char * >( iFlash);
  // and try again... so will read the first byte of the insert. 
  return read();
}

byte RingStream::readRawByte() {
  byte b=_buffer[_pos_read];
  _pos_read++;
  if (_pos_read==_len) _pos_read=0;
  _overflow=false;
  return b;
}

int RingStream::count() {
  return (readRawByte()<<8) | readRawByte(); 
  }

int RingStream::freeSpace() {
  // allow space for client flag and length bytes
  if (_pos_read>_pos_write) return _pos_read-_pos_write-3;
  else return _len - _pos_write + _pos_read-3;  
}


// mark start of message with client id (0...9)
void RingStream::mark(uint8_t b) {
    //DIAG(F("RS mark client %d at %d core %d"), b, _pos_write, xPortGetCoreID());
    _ringClient = b;
    _mark=_pos_write;
    write(b); // client id
    write((uint8_t)0);  // count MSB placemarker
    write((uint8_t)0);  // count LSB placemarker
    _count=0;
}

// peekTargetMark is used by the parser stash routines to know which client
// to send a callback response to some time later. 
uint8_t RingStream::peekTargetMark() {
  return _ringClient;
}

void RingStream::info() {
  DIAG(F("Info len=%d count=%d pr=%d pw=%d m=%d"),_len, _count,_pos_read,_pos_write,_mark);
}

bool RingStream::commit() {
  _flashInsert=NULL; // prepared for first read
  if (_overflow) {
        //DIAG(F("RingStream(%d) commit(%d) OVERFLOW"),_len, _count);
        // just throw it away 
        _pos_write=_mark;
        _overflow=false;
        return false; // commit failed
  }
  if (_count==0) {
    //DIAG(F("RS commit count=0 rewind back to %d core %d"), _mark, xPortGetCoreID());
    // ignore empty response
    _pos_write=_mark;
    _ringClient = NO_CLIENT;         //XXX make else clause later
    return true; // true=commit ok
  }
  // Go back to the _mark and inject the count 1 byte later
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=highByte(_count);
  _mark++;
  if (_mark==_len) _mark=0;
  _buffer[_mark]=lowByte(_count);
  _ringClient = NO_CLIENT;
  return true; // commit worked
}
void RingStream::flush() {
  _pos_write=0;
  _pos_read=0;
  _buffer[0]=0;
  _flashInsert=NULL; // prepared for first read
  _ringClient = NO_CLIENT;
}
  
