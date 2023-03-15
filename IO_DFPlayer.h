/*
 *  © 2022, Neil McKechnie. All rights reserved.
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
 * Writing an analogue value 1-2999 to the first pin (3500) will play the numbered file from the SD card;
 * Writing an analogue value 0 to the first pin (3500) will stop the file playing;
 * Writing an analogue value 0-30 to the second pin (3501) will set the volume;
 * Writing a digital value of 1 to a pin will play the file corresponding to that pin, e.g.
   the first file will be played by setting pin 3500, the second by setting pin 3501 etc.;
 * Writing a digital value of 0 to any pin will stop the player;
 * Reading a digital value from any pin will return true(1) if the player is playing, false(0) otherwise.
 * 
 * From EX-RAIL, the following commands may be used:
 *   SET(3500)      -- starts playing the first file (file 1) on the SD card
 *   SET(3501)      -- starts playing the second file (file 2) on the SD card
 *   etc.
 *   RESET(3500)    -- stops all playing on the player
 *   WAITFOR(3500)  -- wait for the file currently being played by the player to complete
 *   SERVO(3500,2,Instant)  -- plays file 2 at current volume
 *   SERVO(3501,20,Instant)   -- Sets the volume to 20
 * 
 * NB The DFPlayer's serial lines are not 5V safe, so connecting the Arduino TX directly 
 * to the DFPlayer's RX terminal will cause lots of noise over the speaker, or worse.
 * A 1k resistor in series with the module's RX terminal will alleviate this.
 * 
 * Files on the SD card are numbered according to their order in the directory on the 
 * card (as listed by the DIR command in Windows).  This may not match the order of the files 
 * as displayed by Windows File Manager, which sorts the file names.  It is suggested that
 * files be copied into an empty SDcard in the desired order, one at a time.
 */

#ifndef IO_DFPlayer_h
#define IO_DFPlayer_h

#include "IODevice.h"

class DFPlayer : public IODevice {
private: 
  const uint8_t MAXVOLUME=30;
  HardwareSerial *_serial;
  bool _playing = false;
  uint8_t _inputIndex = 0;
  unsigned long _commandSendTime; // Allows timeout processing
  uint8_t _requestedVolumeLevel = MAXVOLUME;
  uint8_t _currentVolume = MAXVOLUME;
  int _requestedSong = -1;  // -1=none, 0=stop, >0=file number
  
public:
 
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial) {
    if (checkNoOverlap(firstVpin,nPins)) new DFPlayer(firstVpin, nPins, serial);
  }

protected:
  // Constructor
  DFPlayer(VPIN firstVpin, int nPins, HardwareSerial &serial) :
    IODevice(firstVpin, nPins),
    _serial(&serial) 
  {
    addDevice(this);
  }

  void _begin() override {
    _serial->begin(9600, SERIAL_8N1); // 9600baud, no parity, 1 stop bit
    // Flush any data in input queue
    while (_serial->available()) _serial->read();
    _deviceState = DEVSTATE_INITIALISING;

    // Send a query to the device to see if it responds
    sendPacket(0x42); 
    _commandSendTime = micros();
  }

  void _loop(unsigned long currentMicros) override {
    // Check for incoming data on _serial, and update busy flag accordingly.
    // Expected message is in the form "7E FF 06 3D xx xx xx xx xx EF"
    while (_serial->available()) {
      int c = _serial->read();
      if (c == 0x7E && _inputIndex == 0) 
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

    // Check if the initial prompt to device has timed out.  Allow 5 seconds
    if (_deviceState == DEVSTATE_INITIALISING && currentMicros - _commandSendTime > 5000000UL) {
      DIAG(F("DFPlayer device not responding on serial port"));
      _deviceState = DEVSTATE_FAILED;
    }

    // When two commands are sent in quick succession, the device will often fail to 
    // execute one.  Testing has indicated that a delay of 100ms or more is required
    // between successive commands to get reliable operation.
    // If 100ms has elapsed since the last thing sent, then check if there's some output to do.
    if (currentMicros - _commandSendTime > 100000UL) {
      if (_currentVolume > _requestedVolumeLevel) {
        // Change volume before changing song if volume is reducing.
        _currentVolume = _requestedVolumeLevel;
        sendPacket(0x06, _currentVolume);
        _commandSendTime = currentMicros;
      } else if (_requestedSong > 0) {
        // Change song
        sendPacket(0x03, _requestedSong);
        _requestedSong = -1;
        _commandSendTime = currentMicros;
      } else if (_requestedSong == 0) {
        sendPacket(0x0e);  // Pause playing
        _requestedSong = -1;
        _commandSendTime = currentMicros;
      } else if (_currentVolume < _requestedVolumeLevel) {
        // Change volume after changing song if volume is increasing.
        _currentVolume = _requestedVolumeLevel;
        sendPacket(0x06, _currentVolume);
        _commandSendTime = currentMicros;
      }
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
      _requestedSong = pin+1;
      _playing = true;
    } else {
      // Value 0, stop playing
      #ifdef DIAG_IO
      DIAG(F("DFPlayer: Stop"));
      #endif
      _requestedSong = 0;  // No song
      _playing = false;
    }
  }

  // WriteAnalogue on first pin uses the nominated value as a file number to start playing, if file number > 0.
  // Volume may be specified as second parameter to writeAnalogue.
  // If value is zero, the player stops playing.  
  // WriteAnalogue on second pin sets the output volume.
  //
  void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
    uint8_t pin = vpin - _firstVpin;
 
    #ifdef DIAG_IO
    DIAG(F("DFPlayer: VPIN:%d FileNo:%d Volume:%d"), vpin, value, volume);
    #endif

    // Validate parameter.
    if (volume > MAXVOLUME) volume = MAXVOLUME;

    if (pin == 0) {
      // Play track 
      if (value > 0) {
        if (volume > 0)
          _requestedVolumeLevel = volume;
        _requestedSong = value;
        _playing = true;
      } else {
        _requestedSong = 0; // stop playing
        _playing = false;
      }
    } else if (pin == 1) {
      // Set volume (0-30)
      _requestedVolumeLevel = value;  
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

    // Output the command
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
