/*
 *  © 2023, Neil McKechnie. All rights reserved.
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
 *   In halSetup function within myHal.cpp:
 *       DFPlayer::create(3500, 5, Serial1);
 *   or in myAutomation.h:
 *       HAL(DFPlayer, 3500, 5, Serial1)
 * 
 * Writing an analogue value 1-2999 to the first pin (3500) will play the numbered file from the
 * SD card; e.g. a value of 1 will play the first file, 2 for the second file etc.
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
 * 
 * The driver now polls the device for its current status every second.  Should the device
 * fail to respond it will be marked off-line and its busy indicator cleared, to avoid
 * lock-ups in automation scripts that are executing for a WAITFOR().
 *
 * *********************************************************************************************
 * 2023, Added NXP SC16IS752 I2C Dual UART to enable the DFPlayer connection over the I2C bus
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be needed as the RX Fifo holds the reply
 *
 *
 */

#ifndef IO_I2CDFPlayer_h
#define IO_I2CDFPlayer_h

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

//#define DIAG_I2CDFplayer
//#define DIAG_I2CDFplayer_data
//#define DIAG_I2CDFplayer_reg

class I2CDFPlayer : public IODevice {
private: 
  const uint8_t MAXVOLUME=30;
  bool _playing = false;
  uint8_t _inputIndex = 0;
  unsigned long _commandSendTime; // Time (us) that last transmit took place.
  unsigned long _timeoutTime;
  uint8_t _recvCMD;  // Last received command code byte
  bool _awaitingResponse = false;
  uint8_t _requestedVolumeLevel = MAXVOLUME;
  uint8_t _currentVolume = MAXVOLUME;
  int _requestedSong = -1;  // -1=none, 0=stop, >0=file number
  uint8_t _repeat;
  uint8_t _previousCmd = true;
  // SC16IS752 defines
  I2CAddress _I2CAddress;
  I2CRB _rb;
  uint8_t _UART_CH;
  // Communication parameters for the DFPlayer are fixed at 8 bit, No parity, 1 stopbit
  uint8_t WORD_LEN = 0x03;    // Value LCR bit 0,1
  uint8_t STOP_BIT = 0x00;    // Value LCR bit 2 
  uint8_t PARITY_ENA = 0x00;  // Value LCR bit 3
  uint8_t PARITY_TYPE = 0x00; // Value LCR bit 4
  uint32_t BAUD_RATE = 9600;
  uint8_t PRESCALER = 0x01;   // Value MCR bit 7
  uint8_t TEMP_REG_VAL = 0x00;
  uint8_t FIFO_RX_LEVEL = 0x00;
  uint8_t RX_BUFFER = 0x00; // nr of bytes copied into _inbuffer
  uint8_t FIFO_TX_LEVEL = 0x00;
  uint8_t _outbuffer [11]; // DFPlayer command is 10 bytes + 1 byte register address & UART channel
  uint8_t _inbuffer[10]; // expected DFPlayer return 10 bytes
  
  unsigned long SC16IS752_XTAL_FREQ = 1843200; // May need to change oscillator frequency to 14.7456Mhz (14745600) to allow for higher baud rates
  //unsigned long SC16IS752_XTAL_FREQ = 14745600; // Support for higher baud rates

public:
  // Constructor
  I2CDFPlayer(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t UART_CH){
    _firstVpin = firstVpin;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    _UART_CH = UART_CH;  
    addDevice(this);
   } 
  
public:
   static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t UART_CH) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new I2CDFPlayer(firstVpin, nPins, i2cAddress, UART_CH);
  }

  void _begin() override {
    // check if SC16IS752 exist first, initialize and then resume DFPlayer init via SC16IS752
    I2CManager.begin();
    I2CManager.setClock(1000000);
    if (I2CManager.exists(_I2CAddress)){
      DIAG(F("SC16IS752 I2C:%s UART detected"), _I2CAddress.toString());
      Init_SC16IS752(); // Initialize UART
      if (_deviceState == DEVSTATE_FAILED){
        DIAG(F("SC16IS752 I2C:%s UART initialization failed"), _I2CAddress.toString());
        }
      } else {
         DIAG(F("SC16IS752 I2C:%s UART not detected"), _I2CAddress.toString());
        }
      #if defined(DIAG_IO)
      _display();
      #endif
      // Now init DFPlayer
      // Send a query to the device to see if it responds
      _deviceState = DEVSTATE_INITIALISING; 
      sendPacket(0x42);
      _timeoutTime = micros() + 5000000UL;  // 5 second timeout      
      _awaitingResponse = true;
     }
  
  
  void _loop(unsigned long currentMicros) override {
    // Read responses from device
    uint8_t status = _rb.status;
    if (status == I2C_STATUS_PENDING) return;  // Busy, so don't do anything
    if (status == I2C_STATUS_OK) { 
      processIncoming(currentMicros);
      // Check if a command sent to device has timed out.  Allow 0.5 second for response   
      if (_awaitingResponse && (int32_t)(currentMicros - _timeoutTime) > 0) {
        DIAG(F("I2CDFPlayer:%s, DFPlayer not responding on UART channel: 0x%x"), _I2CAddress.toString(), _UART_CH);
        _deviceState = DEVSTATE_FAILED;
       _awaitingResponse = false;
        _playing = false;
      }
    }

    status = _rb.status;
    if (status == I2C_STATUS_PENDING) return;  // Busy, try next time
    if (status == I2C_STATUS_OK) {
     // Send any commands that need to go.
      processOutgoing(currentMicros);
     }
    delayUntil(currentMicros + 10000); // Only enter every 10ms    
  }

 
  // Check for incoming data on _serial, and update busy flag and other state accordingly
 
  void processIncoming(unsigned long currentMicros) {
    // Expected message is in the form "7E FF 06 3D xx xx xx xx xx EF"
    RX_fifo_lvl();
    if (FIFO_RX_LEVEL >= 10) {      
      #ifdef DIAG_I2CDFplayer
        DIAG(F("I2CDFPlayer: %s Retrieving data from RX Fifo on UART_CH: 0x%x FIFO_RX_LEVEL: %d"),_I2CAddress.toString(), _UART_CH, FIFO_RX_LEVEL); 
      #endif
      _outbuffer[0] = REG_RHR << 3 | _UART_CH << 1;
      // Only copy 10 bytes from RX FIFO, there maybe additional partial return data after a track is finished playing in the RX FIFO
      I2CManager.read(_I2CAddress, _inbuffer, 10, _outbuffer, 1); // inbuffer[] has the data now
      //delayUntil(currentMicros + 10000); // Allow time to get the data
      RX_BUFFER = 10; // We have copied 10 bytes from RX FIFO to _inbuffer
        #ifdef DIAG_I2CDFplayer_data
          DIAG(F("SC16IS752: At I2C: %s, UART channel: 0x%x, RX FIFO Data"), _I2CAddress.toString(), _UART_CH);
          for (int i = 0; i < sizeof _inbuffer; i++){
            DIAG(F("SC16IS752: Data _inbuffer[0x%x]: 0x%x"), i, _inbuffer[i]);  
          }
        #endif       
    } else {
        FIFO_RX_LEVEL = 0; //set to 0, we'll read a fresh FIFO_RX_LEVEL next time
        return; // No data or not enough data in rx fifo, check again next time around
      }

/*
    #ifdef DIAG_I2CDFplayer
     if (FIFO_RX_LEVEL > 10) {
        DIAG(F("I2CDFPlayer: %s FIFO_RX_LEVEL: %d"),_I2CAddress.toString(), FIFO_RX_LEVEL); 
     }
    #endif
*/
    
    bool ok = false;
    //DIAG(F("I2CDFPlayer: RX_BUFFER: %d"), RX_BUFFER);
    while (RX_BUFFER != 0) {
      int c = _inbuffer[_inputIndex]; // Start at 0, increment to FIFO_RX_LEVEL
      switch (_inputIndex) {
        case 0:
          if (c == 0x7E) ok = true;
          break;
        case 1:
          if (c == 0xFF) ok = true;
          break;
        case 2:
          if (c== 0x06) ok = true;
          break;
        case 3:
          _recvCMD = c; // CMD byte
          ok = true;
          break;
        case 6:
          switch (_recvCMD) {
            DIAG(F("I2CDFPlayer: %s, _recvCMD: 0x%x _awaitingResponse: 0x0%x"),_I2CAddress.toString(), _recvCMD, _awaitingResponse);
            case 0x42:
              // Response to status query
              _playing = (c != 0);              
              // Mark the device online and cancel timeout
              if (_deviceState==DEVSTATE_INITIALISING) {
                _deviceState = DEVSTATE_NORMAL;
                #ifdef DIAG_I2CDFplayer
                 DIAG(F("I2CDFPlayer: %s, UART_CH: 0x0%x, _deviceState: 0x0%x"),_I2CAddress.toString(), _UART_CH, _deviceState);
                #endif 
                #ifdef DIAG_IO
                _display();
                #endif
              }
              _awaitingResponse = false;
              break;
            case 0x3d:            
              // End of play
              if (_playing) {
                #ifdef DIAG_IO
                  DIAG(F("I2CDFPlayer: Finished"));
                #endif
                _playing = false;
              }
              break;
            case 0x40:
              // Error codes; 1: Module Busy
              DIAG(F("I2CDFPlayer: Error %d returned from device"), c);
              _playing = false;
              break;
          }
          ok = true;
          break;
        case 4: case 5: case 7: case 8: 
          ok = true;  // Skip over these bytes in message.
          break;
        case 9:
          if (c==0xef) {
            // Message finished
          }
          break;
        default:
          break;
      }
      if (ok){
        _inputIndex++;  // character as expected, so increment index
        RX_BUFFER --; // Decrease FIFO_RX_LEVEL with each character read from _inbuffer[_inputIndex]
      } else {
        _inputIndex = 0;  // otherwise reset.
        RX_BUFFER = 0;
      }
    }
    RX_BUFFER = 0; //Set to 0, we'll read a new RX FIFO level again
  }

  // Send any commands that need to be sent
  void processOutgoing(unsigned long currentMicros) {
    // When two commands are sent in quick succession, the device will often fail to 
    // execute one.  Testing has indicated that a delay of 100ms or more is required
    // between successive commands to get reliable operation.
    // If 100ms has elapsed since the last thing sent, then check if there's some output to do.
    if (((int32_t)currentMicros - _commandSendTime) > 100000) { 
      if (_currentVolume > _requestedVolumeLevel) {
        // Change volume before changing song if volume is reducing.
        _currentVolume = _requestedVolumeLevel;
        sendPacket(0x06, _currentVolume);
      } else if (_requestedSong > 0) {
        // Change song
        sendPacket(0x03, _requestedSong);
        _requestedSong = -1;    
      } else if (_requestedSong == 0) {
        sendPacket(0x16);  // Stop playing
        _requestedSong = -1;
      } else if (_currentVolume < _requestedVolumeLevel) {
        // Change volume after changing song if volume is increasing.
        _currentVolume = _requestedVolumeLevel;
        sendPacket(0x06, _currentVolume);
      } else if ((int32_t)currentMicros - _commandSendTime > 1000000) {
        // Poll device every second that other commands aren't being sent,
        // to check if it's still connected and responding.
        sendPacket(0x42); 
        if (!_awaitingResponse) {
          _timeoutTime = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds
          _awaitingResponse = true;
        }
      }
    }  
  }

  // Write with value 1 starts playing a song.  The relative pin number is the file number.
  // Write with value 0 stops playing.
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    if (value) {
      // Value 1, start playing
      #ifdef DIAG_IO
        DIAG(F("I2CDFPlayer: Play %d"), pin+1);
      #endif
      _requestedSong = pin+1;
      _playing = true;
    } else {
      // Value 0, stop playing
      #ifdef DIAG_IO
        DIAG(F("I2CDFPlayer: Stop"));
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
  //void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
  void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t cmd=0) override { 
    if (_deviceState == DEVSTATE_FAILED) return;
    uint8_t pin = vpin - _firstVpin;
    #ifdef DIAG_IO
      DIAG(F("I2CDFPlayer: VPIN:%u FileNo:%d Volume:%d Repeat:0x0%x"), vpin, value, volume, cmd);
    #endif
    // Validate parameter.
    if (volume > MAXVOLUME) volume = MAXVOLUME;

    if (pin == 0) {
      // Play track
      if (value > 0) {
        if (volume > 0)
          _requestedVolumeLevel = volume;
        _requestedSong = value;
        if (cmd = 1){ // check for Repeat playback of song          
          _repeat = true;
        } else {          
          _repeat = false;
        }
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
    if (_deviceState == DEVSTATE_FAILED) return false;
    return _playing;
  }

  void _display() override {
    DIAG(F("I2CDFPlayer Configured on Vpins:%u-%u %S"), _firstVpin, _firstVpin+_nPins-1,
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
    FIFO_TX_LEVEL = 0; // Reset FIFO_TX_LEVEL    
    uint8_t out[] = {
        0x7E,
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

      // Prepend the DFPlayer command with REG address and UART Channel in _outbuffer
      _outbuffer[0] = REG_THR << 3 | _UART_CH << 1; //TX FIFO and UART Channel      
      for ( int i = 1; i < sizeof(out)+1 ; i++){
        _outbuffer[i] = out[i-1];
      }

      #ifdef DIAG_I2CDFplayer_data
       DIAG(F("SC16IS752: I2C: %s Sent packet function"), _I2CAddress.toString());
       for (int i = 0; i < sizeof _outbuffer; i++){
        DIAG(F("SC16IS752: Data _outbuffer[0x%x]: 0x%x"), i, _outbuffer[i]);  
       }
      #endif
      
    TX_fifo_lvl();
    if(FIFO_TX_LEVEL > 0){ //FIFO is empty
      I2CManager.write(_I2CAddress, _outbuffer, sizeof(_outbuffer), &_rb);
      //I2CManager.write(_I2CAddress, _outbuffer, sizeof(_outbuffer));
      #ifdef DIAG_I2CDFplayer
       DIAG(F("SC16IS752: I2C: %s data transmit complete on UART: 0x%x"), _I2CAddress.toString(), _UART_CH);
      #endif
    } else {
      DIAG(F("I2CDFPlayer at: %s, TX FIFO not empty on UART: 0x%x"), _I2CAddress.toString(), _UART_CH);
      _deviceState = DEVSTATE_FAILED; // This should not happen      
    }
    _commandSendTime = micros();
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

  // SC16IS752 functions
  // Initialise SC16IS752 only for this channel
  // First a software reset
  // Enable FIFO and clear TX & RX FIFO
  // Need to set the following registers
  // LCR bit 7=0 divisor latch (clock division registers DLH & DLL, they store 16 bit divisor), 
  //     WORD_LEN, STOP_BIT, PARITY_ENA and PARITY_TYPE
  // MCR bit 7=0 clock divisor devide-by-1 clock input
  // DLH most significant part of divisor
  // DLL least significant part of divisor
  //
  // BAUD_RATE, WORD_LEN, STOP_BIT, PARITY_ENA and PARITY_TYPE have been defined and initialized
  // 
  void Init_SC16IS752(){ // Return value is in _deviceState
    #ifdef DIAG_I2CDFplayer
      DIAG(F("SC16IS752: Initialize I2C: %s , UART Ch: 0x%x"), _I2CAddress.toString(),  _UART_CH);      
    #endif
    uint16_t _divisor = (SC16IS752_XTAL_FREQ / PRESCALER) / (BAUD_RATE * 16);
    TEMP_REG_VAL = 0x08; // UART Software reset
    UART_WriteRegister(REG_IOCONTROL, TEMP_REG_VAL);
    TEMP_REG_VAL = 0x07; // Reset FIFO, clear RX & TX FIFO
    UART_WriteRegister(REG_FCR, TEMP_REG_VAL);
    TEMP_REG_VAL = 0x00; // Set MCR to all 0, includes Clock divisor
    UART_WriteRegister(REG_MCR, TEMP_REG_VAL);
    TEMP_REG_VAL = 0x80 | WORD_LEN | STOP_BIT | PARITY_ENA | PARITY_TYPE;
    UART_WriteRegister(REG_LCR, TEMP_REG_VAL); // Divisor latch enabled
    UART_WriteRegister(REG_DLL, (uint8_t)_divisor);  // Write DLL
    UART_WriteRegister(REG_DLH, (uint8_t)(_divisor >> 8)); // Write DLH
    UART_ReadRegister(REG_LCR);
    TEMP_REG_VAL = _inbuffer[0] & 0x7F; // Disable Divisor latch enabled bit
    UART_WriteRegister(REG_LCR, TEMP_REG_VAL); // Divisor latch disabled  

    uint8_t status = _rb.status;
    if (status != I2C_STATUS_OK) {
      DIAG(F("SC16IS752: I2C: %s failed %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
      _deviceState = DEVSTATE_FAILED;
    } else {
      #ifdef DIAG_IO
       DIAG(F("SC16IS752: I2C: %s, _deviceState: %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
      #endif
     _deviceState = DEVSTATE_NORMAL; // If I2C state is OK, then proceed to initialize DFPlayer 
    }
  }

  
  // Read the Receive FIFO Level register (RXLVL), return a single unsigned integer
  // of nr of characters in the RX FIFO, bit 6:0, 7 not used, set to zero
  // value from 0 (0x00) to 64 (0x40) Only display if RX FIFO has data
  void RX_fifo_lvl(){
    UART_ReadRegister(REG_RXLV);
    FIFO_RX_LEVEL = _inbuffer[0];
    #ifdef DIAG_I2CDFplayer
    if (FIFO_RX_LEVEL > 0){
    //  DIAG(F("SC16IS752: At I2C: %s, UART channel: 0x%x, FIFO_RX_LEVEL: 0d%d"), _I2CAddress.toString(), _UART_CH, _inbuffer[0]);
    }
    #endif   
  }

  // Read the Tranmit FIFO Level register (TXLVL), return a single unsigned integer
  // of nr characters free in the TX FIFO, bit 6:0, 7 not used, set to zero
  // value from 0 (0x00) to 64 (0x40)
  //
  void TX_fifo_lvl(){
    UART_ReadRegister(REG_TXLV);
    FIFO_TX_LEVEL = _inbuffer[0];
    #ifdef DIAG_I2CDFplayer
    //  DIAG(F("SC16IS752: At I2C: %s, UART channel: 0x%x, FIFO_TX_LEVEL: 0d%d"), _I2CAddress.toString(), _UART_CH, FIFO_TX_LEVEL);
    #endif 
  }


  //void UART_WriteRegister(I2CAddress _I2CAddress, uint8_t _UART_CH, uint8_t UART_REG, uint8_t Val, I2CRB &_rb){
  void UART_WriteRegister(uint8_t UART_REG, uint8_t Val){
    _outbuffer[0] = UART_REG << 3 | _UART_CH << 1;
    _outbuffer[1] = Val;
    #ifdef DIAG_I2CDFplayer_reg
      DIAG(F("SC16IS752: Write register at I2C: %s, UART channel: 0x%x, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _UART_CH, UART_REG, _outbuffer[1]);
    #endif
    I2CManager.write(_I2CAddress, _outbuffer, 2);
  }

 
  void UART_ReadRegister(uint8_t UART_REG){
     _outbuffer[0] = UART_REG << 3 | _UART_CH << 1; // _outbuffer[0] has now UART_REG and UART_CH
     I2CManager.read(_I2CAddress, _inbuffer, 1, _outbuffer, 1);    
    // _inbuffer has the REG data
    #ifdef DIAG_I2CDFplayer_reg
      DIAG(F("SC16IS752: Read register at I2C: %s, UART channel: 0x%x, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _UART_CH, UART_REG, _inbuffer[0]);
    #endif
  }

// SC16IS752 General register set (from the datasheet)
enum : uint8_t{
    REG_RHR       = 0x00, // FIFO Read
    REG_THR       = 0x00, // FIFO Write
    REG_IER       = 0x01, // Interrupt Enable Register R/W
    REG_FCR       = 0x02, // FIFO Control Register Write
    REG_IIR       = 0x02, // Interrupt Identification Register Read
    REG_LCR       = 0x03, // Line Control Register R/W
    REG_MCR       = 0x04, // Modem Control Register R/W
    REG_LSR       = 0x05, // Line Status Register Read
    REG_MSR       = 0x06, // Modem Status Register Read
    REG_SPR       = 0x07, // Scratchpad Register R/W
    REG_TCR       = 0x06, // Transmission Control Register R/W
    REG_TLR       = 0x07, // Trigger Level Register R/W    
    REG_TXLV      = 0x08, // Transmitter FIFO Level register Read
    REG_RXLV      = 0x09, // Receiver FIFO Level register Read
    REG_IODIR     = 0x0A, // Programmable I/O pins Direction register R/W
    REG_IOSTATE   = 0x0B, // Programmable I/O pins State register R/W
    REG_IOINTENA  = 0x0C, // I/O Interrupt Enable register R/W
    REG_IOCONTROL = 0x0E, // I/O Control register R/W
    REG_EFCR      = 0x0F, // Extra Features Control Register R/W
  };

// SC16IS752 Special register set
enum : uint8_t{
    REG_DLL       = 0x00, // Division registers R/W
    REG_DLH       = 0x01, // Division registers R/W
  };

// SC16IS752 Enhanced regiter set
enum : uint8_t{
    REG_EFR       = 0X02, // Enhanced Features Register R/W
    REG_XON1      = 0x04, // R/W
    REG_XON2      = 0x05, // R/W
    REG_XOFF1     = 0x06, // R/W
    REG_XOFF2     = 0x07, // R/W
  };

};

#endif // IO_I2CDFPlayer_h
