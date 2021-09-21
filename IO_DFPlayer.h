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
 * DFPlayer is an MP3 player module with an SD card holder.  It also has an integrated
 * amplifier, so it only needs a power supply and a speaker.
 * 
 * This driver allows the device to be controlled through IODevice::write() and 
 * IODevice::writeAnalogue() calls.
 * 
 * The driver is configured as follows:
 * 
 *       DFPlayer::create(firstVpin, nPins, Serialn);
 * 
 * Where firstVpin is the first vpin reserved for reading the device,
 *       nPins is the number of pins to be allocated (max 5)
 *   and Serialn is the name of the Serial port connected to the DFPlayer (e.g. Serial1).
 * 
 * Example:
 *   In mySetup function within mySetup.cpp:
 *       DFPlayer::create(3500, 5, Serial1);
 * 
 * Writing an analogue value 0-2999 to the first pin will select a numbered file from the SD card;
 * Writing an analogue value 0-30 to the second pin will set the volume of the output;
 * Writing a digital value to the first pin will play or stop the file;
 * Reading a digital value from any pin will return true(1) if the player is playing, false(0) otherwise.
 * 
 * From EX-RAIL, the following commands may be used:
 *   SET(3500)      -- starts playing the first file on the SD card
 *   SET(3501)      -- starts playing the second file on the SD card
 *   etc.
 *   RESET(3500)    -- stops all playing on the player
 *   WAITFOR(3500)  -- wait for the file currently being played by the player to complete
 *   SERVO(3500,23,0)  -- plays file 23 at current volume
 *   SERVO(3500,23,30)  -- plays file 23 at volume 30 (maximum)
 *   SERVO(3501,20,0)   -- Sets the volume to 20
 * 
 * NB The DFPlayer's serial lines are not 5V safe, so connecting the Arduino TX directly 
 * to the DFPlayer's RX terminal will cause lots of noise over the speaker, or worse.
 * A 1k resistor in series with the module's RX terminal will alleviate this.
 */

#ifndef IO_DFPlayer_h
#define IO_DFPlayer_h

#include "IODevice.h"

class DFPlayer : public IODevice {
private: 
  HardwareSerial *_serial;
  bool _playing = false;
  uint8_t _inputIndex = 0;
  unsigned long _commandSendTime; // Allows timeout processing

public:
  // Constructor
  DFPlayer(VPIN firstVpin, int nPins, HardwareSerial &serial) :
    IODevice(firstVpin, nPins),
    _serial(&serial) 
  {
    addDevice(this);
  }

  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial) {
    new DFPlayer(firstVpin, nPins, serial);
  }

protected:
  void _begin() override {
    _serial->begin(9600);
    _deviceState = DEVSTATE_INITIALISING;

    // Send a query to the device to see if it responds
    sendPacket(0x42); 
    _commandSendTime = micros();
  }

  void _loop(unsigned long currentMicros) override {
    // Check for incoming data on _serial, and update busy flag accordingly.
    // Expected message is in the form "7F FF 06 3D xx xx xx xx xx EF"
    while (_serial->available()) {
      int c = _serial->read();
      if (c == 0x7E) 
        _inputIndex = 1;
      else if ((c==0xFF && _inputIndex==1)
            || (c==0x3D && _inputIndex==3) 
            || (_inputIndex >=4 && _inputIndex <= 8))
        _inputIndex++;
      else if (c==0x06 && _inputIndex==2) { 
        // Valid message prefix, so consider the device online
        if (_deviceState==DEVSTATE_INITIALISING) {
          _deviceState = DEVSTATE_NORMAL;
          #ifdef DIAG_IO
          _display();
          #endif
        }
        _inputIndex++;
      } else if (c==0xEF && _inputIndex==9) {
        // End of play
        if (_playing) {
          #ifdef DIAG_IO
          DIAG(F("DFPlayer: Finished"));
          #endif
          _playing = false;
        }
        _inputIndex = 0;
      } else 
        _inputIndex = 0;  // Unrecognised character sequence, start again!
    }
    // Check if the initial prompt to device has timed out.  Allow 1 second
    if (_deviceState == DEVSTATE_INITIALISING && currentMicros - _commandSendTime > 1000000UL) {
      DIAG(F("DFPlayer device not responding on serial port"));
      _deviceState = DEVSTATE_FAILED;
    }
    delayUntil(currentMicros + 10000); // Only enter every 10ms
  }

  // Write with value 1 starts playing a song.  The relative pin number is the file number.
  // Write with value 0 stops playing.
  void _write(VPIN vpin, int value) override {
    int pin = vpin - _firstVpin;
    if (value) {
      // Value 1, start playing
      #ifdef DIAG_IO
      DIAG(F("DFPlayer: Play %d"), pin+1);
      #endif
      sendPacket(0x03, pin+1);
      _playing = true;
    } else {
      // Value 0, stop playing
      #ifdef DIAG_IO
      DIAG(F("DFPlayer: Stop"));
      #endif
      sendPacket(0x16);
      _playing = false;
    }
  }

  // WriteAnalogue on first pin uses the nominated value as a file number to start playing, if file number > 0.
  // Volume may be specified as second parameter to writeAnalogue.
  // If value is zero, the player stops playing.  
  // WriteAnalogue on second pin sets the output volume.
  void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
    uint8_t pin = vpin - _firstVpin;
 
    // Validate parameter.
    volume = min(30,volume);

    if (pin == 0) {
      // Play track 
      if (value > 0) {
        #ifdef DIAG_IO
        DIAG(F("DFPlayer: Play %d"), value);
        #endif
        sendPacket(0x03, value); // Play track
        _playing = true;
        if (volume > 0) {
          #ifdef DIAG_IO
          DIAG(F("DFPlayer: Volume %d"), volume);
          #endif
          sendPacket(0x06, volume);  // Set volume
        }
      } else {
        #ifdef DIAG_IO
        DIAG(F("DFPlayer: Stop"));
        #endif
        sendPacket(0x16); // Stop play
        _playing = false;
      }
    } else if (pin == 1) {
      // Set volume (0-30)
      if (value > 30) value = 30;
      else if (value < 0) value = 0;
      #ifdef DIAG_IO
      DIAG(F("DFPlayer: Volume %d"), value);
      #endif
      sendPacket(0x06, value);      
    }
  }

  // A read on any pin indicates whether the player is still playing.
  int _read(VPIN) override {
    return _playing;
  }

  void _display() override {
    DIAG(F("DFPlayer Configured on Vpins:%d-%d %S"), _firstVpin, _firstVpin+_nPins-1,
      (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
  
private:
  // 7E FF 06 0F 00 01 01 xx xx EF
  // 0	->	7E is start code
  // 1	->	FF is version
  // 2	->	06 is length
  // 3	->	0F is command
  // 4	->	00 is no receive
  // 5~6	->	01 01 is argument
  // 7~8	->	checksum = 0 - ( FF+06+0F+00+01+01 )
  // 9	->	EF is end code

  void sendPacket(uint8_t command, uint16_t arg = 0)
  {
    uint8_t out[] = { 0x7E,
        0xFF,
        06,
        command,
        00,
        static_cast<uint8_t>(arg >> 8),
        static_cast<uint8_t>(arg & 0x00ff),
        00,
        00,
        0xEF };

    setChecksum(out);

    _serial->write(out, sizeof(out));
  }

  uint16_t calcChecksum(uint8_t* packet)
  {
    uint16_t sum = 0;
    for (int i = 1; i < 7; i++)
    {
      sum += packet[i];
    }
    return -sum;
  }

  void setChecksum(uint8_t* out)
  {
    uint16_t sum = calcChecksum(out);

    out[7] = (sum >> 8);
    out[8] = (sum & 0xff);
  }
};

#endif // IO_DFPlayer_h
