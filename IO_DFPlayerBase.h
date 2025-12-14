   /*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  © 2025, Henk-Jan van der Klis
 *  © 2025, Chris Harlow
 * 
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
 * This file contains basic DFPalyer logic without the communication mentod.
 * Subclasses are expected to implement the communication method, e.g.
 * using HardwareSerial or I2C UART. 
 * 
 * Command queueing is implemented to allow for reliable operation
 * even when the communication method is relatively slow.
 * 
 * Subclass must send whn requested and is given polling time to process incoming data.
 * The subclass must call processIncomingByte() for each incoming byte and it is not
 * expected to buffer complete messages.
 */

#ifndef IO_DFPlayerBase_h
#define IO_DFPlayerBase_h

#include "IODevice.h"
#include "DIAG.h"


class DFPlayerBase : public IODevice {
private: 
  const uint8_t MAXVOLUME=30;
  uint8_t RETRYCOUNT = 0x03;
  bool _playing = false;
  uint8_t _inputIndex = 0;
  unsigned long _commandSendTime; // Time (us) that last transmit took place.
  unsigned long _timeoutTime;
  uint8_t _recvCMD;  // Last received command code byte
  bool _awaitingResponse = false;  
  uint8_t _retryCounter = RETRYCOUNT; // Max retries before timing out
  uint8_t _currentVolume = MAXVOLUME;
  uint8_t _currentFolder = 0x01; // default to folder 01
  
  // 4-entry circular queue for command arguments
  struct CommandEntry {
    uint8_t cmd;
    uint8_t arg1;
    uint8_t arg2;
  };
  static const uint8_t COMMAND_QUEUE_SIZE = 4;
  CommandEntry _commandQueue[COMMAND_QUEUE_SIZE];
  uint8_t _queueHead = 0;  // Points to next slot to write
  uint8_t _queueTail = 0;  // Points to next slot to read
  
  // Add command to queue
  void queuePacket(uint8_t cmd, uint8_t arg1 = 0, uint8_t arg2 = 0) { 
    uint8_t nextHead = (_queueHead + 1) % COMMAND_QUEUE_SIZE;
    if (nextHead != _queueTail) {  // Check if queue is not full
      _commandQueue[_queueHead].cmd = cmd;
      _commandQueue[_queueHead].arg1 = arg1;
      _commandQueue[_queueHead].arg2 = arg2;
      _queueHead = nextHead;
    }
  }
  
  // Get command from queue
  void dequeueCommand() {
    if (_queueHead == _queueTail) return; // empty
      
    buildPacket( _commandQueue[_queueTail].cmd,
                   _commandQueue[_queueTail].arg1,
                   _commandQueue[_queueTail].arg2);
    _queueTail = (_queueTail + 1) % COMMAND_QUEUE_SIZE;
  }
   
   
protected:
  // Constructor
   DFPlayerBase(VPIN firstVpin, int nPins=1): IODevice(firstVpin, nPins) { 
    addDevice(this);
   } 
  
public:

  void _begin() override {
      // Now init DFPlayer
      // Send a query to the device to see if it responds
      _deviceState = DEVSTATE_INITIALISING; 
      queuePacket(0x42,0,0);
      _timeoutTime = micros() + 5000000UL;  // 5 second timeout      
      _awaitingResponse = true; 
      _display();
     }
  
  virtual void transmitCommandBuffer(const uint8_t buffer[], size_t bytes) = 0; // pure virtual function to be implemented in derived class
  virtual bool processIncoming() = 0; // pure virtual function to be implemented in derived class

  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    // Read responses from device, if no responses send a waiting command
    if (!processIncoming()) dequeueCommand(); // Send any one command that need to go.
    delayUntil(currentMicros + 100000); // Only enter every 100ms    
  }

    // Comms modules call this for each incoming byte
  void processIncomingByte(byte c) {
    static const byte EXPECTED_HEADER[] = {0x7E, 0xFF, 0x06};
    // Expected message is in the form "7E FF 06 ss xx xx xx xx EF"
    switch(_inputIndex) {
        case 0: case 1: case 2:
           if (c != EXPECTED_HEADER[_inputIndex]) {
              _inputIndex=0; // keep looking for header
              return;
           }
           break; 
        case 3:
              _recvCMD = c; // CMD byte
              break;
         case 4: case 5:
              // not interested in these bytes 
              break;
         case 6: // response value byte
            switch (_recvCMD) {
               case 0x42: // Response to status query
                  _playing = (c != 0);
                  break;              
                case 0x3d:            
                  _playing = false;
                  break;
                default:
                  // Error codes; 1: Module Busy
                  DIAG(F("DFPlayer: Error code 0x%x value %d"), _recvCMD,c);
                 _playing = false;
                 break;
            }
          break;
        case 7: case 8: 
          break;
        case 9:
          if (c==0xef) { // Message finished
            _retryCounter = RETRYCOUNT; // reset the retry counter as we have received a valid packet
          }
          _inputIndex=0; // reset for next message
          return;
        default: // should not happen
           _inputIndex=0; // reset for next message
           break;
      }
    _inputIndex++;  // character as expected, so increment index
  }


  
  // *************** HAL digital/analog read write ********

  // Write to a digital vPin from SET/RESET or <z > commands
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;

    // Deprecated effects
    // write 1 to vpin will play track number = vpin - firstVpin + 1
    if (value) {
      queuePacket(DF_PLAY, 0x00, vpin - _firstVpin + 1); // play track
      _playing = true;
    }
    else if (_playing) {  // write a 0 stops playing
      queuePacket(0x16, 0x00, 0x00); // stop playing
      _playing = false;
    }
  }


  // A read on any pin indicates if the player is still playing.
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return false;
    return _playing; 
    }

  void _display() override {
    DIAG(F("DFPlayer Configured on Vpins:%u-%u %S"), _firstVpin, _firstVpin+_nPins-1,
      (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
  }
  
private: 
  // DFPlayer command frame
  // 7E FF 06 0F 00 01 01 xx xx EF
  // 0	  ->	7E is start code
  // 1	  ->	FF is version
  // 2	  ->	06 is length
  // 3	  ->	0F is command
  // 4	  ->	00 is no receive
  // 5~6	->	01 01 is argument
  // 7~8	->	checksum = 0 - ( FF+06+0F+00+01+01 )
  // 9	  ->	EF is end code

  // Send packet queues up a command packet to be sent
  
  void buildPacket(uint8_t command, uint8_t arg1 = 0, uint8_t arg2 = 0) {
    uint8_t out[] = {
        0x7E,
        0xFF,
        06,
        command,
        00,
        //static_cast<uint8_t>(arg >> 8),
        //static_cast<uint8_t>(arg & 0x00ff),
        arg1,
        arg2,
        00,
        00,
        0xEF };

   
    int16_t sum = 0;
    for (int i = 1; i < 7; i++)
    {
      sum -= out[i];
    }
    out[7] = (sum >> 8);
    out[8] = (sum & 0xff);

    transmitCommandBuffer(out,sizeof(out)); // pass to superclass for transmission
    }

  
// DFPlayer commands and values
// Declared in this scope
enum  : uint8_t{
    DF_PLAY          = 0x0F,
    DF_VOL           = 0x06,
    DF_FOLDER        = 0x2B, // Not a DFPlayer command, used to set folder nr where audio file is
    DF_REPEATPLAY    = 0x08,
    DF_STOPPLAY      = 0x16,
    DF_EQ            = 0x07, // Set equaliser, require parameter NORMAL, POP, ROCK, JAZZ, CLASSIC or BASS
    DF_RESET         = 0x0C,
    DF_DACON         = 0x1A,
    DF_SETAM         = 0x2A, // Set audio mixer 1 or 2 for this DFPLayer   
    DF_NORMAL        = 0x00, // Equalizer parameters
    DF_POP           = 0x01,
    DF_ROCK          = 0x02,
    DF_JAZZ          = 0x03,
    DF_CLASSIC       = 0x04,
    DF_BASS          = 0x05,
    DF_MAXIMUM_EQ_VALUE = 0x05,    
  };

  protected:
  
  // WriteAnalogue on first pin uses the nominated value as a file number to start playing, if file number > 0.
  // Volume may be specified as second parameter to writeAnalogue.
  // If value is zero, the player stops playing.  
  // WriteAnalogue on second pin sets the output volume.
  //
  // WriteAnalogue to be done on first vpin
  //
  //void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
  void _writeAnalogue(VPIN vpin, int value1, uint8_t value2=0, uint16_t cmd=0) override { 
    if (_deviceState == DEVSTATE_FAILED) return;    
     // Read command and value
      switch (cmd){
        case DF_PLAY:
        case DF_REPEATPLAY:
        case DF_VOL:
          {
          auto volume=min(MAXVOLUME,value2);
          if (volume!=_currentVolume) {
            queuePacket(DF_VOL, 0x00, volume); // set volume first
            _currentVolume = volume;
          }  
          if (cmd!=DF_VOL) queuePacket(cmd,_currentFolder, value1); // then play/repeat file
        }
          break;
       case DF_FOLDER:
        if (value2 <= 0 || value2 > 99) _currentFolder=1; // Range checking, valid values 1-99, else default to 1
        else _currentFolder = value2;
        break;
       case DF_STOPPLAY:
          queuePacket(DF_STOPPLAY); 
          break;
       case DF_EQ:
         { 
            byte eq=value2;
            if (eq<=DF_NORMAL || eq>DF_MAXIMUM_EQ_VALUE) eq=DF_NORMAL; // If out of range, default to NORMAL
            queuePacket(DF_EQ,0,eq);     
          }
          break;        
       case DF_RESET:
           queuePacket(DF_RESET);     
        break; 
       default:
        break;
    }
  }  
  

};

#endif // IO_DFPlayerBase_h
