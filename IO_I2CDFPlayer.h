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
 * This driver is a modified version of the IO_DFPlayer.h file
 * *********************************************************************************************
 * 
 * Dec 2023, Added NXP SC16IS752 I2C Dual UART to enable the DFPlayer connection over the I2C bus
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be
 * needed as the RX Fifo holds the reply 
 * 
 * Jan 2024, Issue with using both UARTs simultaniously, the secod uart seems to work  but the first transmit 
 * corrupt data. This need more analysis and experimenatation. 
 * Will push this driver to the dev branch with the uart fixed to 0 
 * Both SC16IS750 (single uart) and SC16IS752 (dual uart, but only uart 0 is enable)
 * 
 * myHall.cpp configuration syntax:
 * 
 * I2CDFPlayer::create(1st vPin, vPins, I2C address, xtal);
 * 
 * Parameters:
 * 1st vPin     : First virtual pin that EX-Rail can control to play a sound, use PLAYSOUND command (alias of ANOUT)
 * vPins        : Total number of virtual pins allocated (2 vPins are supported, one for each UART)
 *                1st vPin for UART 0, 2nd for UART 1
 * I2C Address  : I2C address of the serial controller, in 0x format
 * xtal         : 0 for 1,8432Mhz, 1 for 14,7456Mhz
 * 
 * The vPin is also a pin that can be read, it indicate if the DFPlayer has finished playing a track
 *
 */

#ifndef IO_I2CDFPlayer_h
#define IO_I2CDFPlayer_h

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

// Debug and diagnostic defines, enable too many will result in slowing the driver
//#define DIAG_I2CDFplayer
//#define DIAG_I2CDFplayer_data
//#define DIAG_I2CDFplayer_reg
//#define DIAG_I2CDFplayer_playing

class I2CDFPlayer : public IODevice {
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
  uint8_t _requestedVolumeLevel = MAXVOLUME;
  uint8_t _currentVolume = MAXVOLUME;
  int _requestedSong = -1;  // -1=none, 0=stop, >0=file number
  bool _repeat = false; // audio file is repeat playing
  uint8_t _previousCmd = true;
  // SC16IS752 defines
  I2CAddress _I2CAddress;
  I2CRB _rb;
  uint8_t _UART_CH=0x00;  // Fix uart ch to 0 for now
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
  bool _playCmd = false;
  bool _volCmd = false;
  bool _folderCmd = false;
  uint8_t _requestedFolder = 0x01; // default to folder 01
  uint8_t _currentFolder = 0x01; // default to folder 01
  bool _repeatCmd = false;
  bool _stopplayCmd = false;
  bool _resetCmd = false;
  bool _eqCmd = false;
  uint8_t _requestedEQValue = DF_NORMAL;
  uint8_t _currentEQvalue = DF_NORMAL; // start equalizer value
  bool _daconCmd = false;
  uint8_t _audioMixer = 0x01; // Default to output amplifier 1
  bool _setamCmd = false; // Set the Audio mixer channel
  uint8_t _outbuffer [11]; // DFPlayer command is 10 bytes + 1 byte register address & UART channel
  uint8_t _inbuffer[10]; // expected DFPlayer return 10 bytes
   
  unsigned long _sc16is752_xtal_freq;
  unsigned long SC16IS752_XTAL_FREQ_LOW = 1843200; // To support cheap eBay/AliExpress SC16IS752 boards
  unsigned long SC16IS752_XTAL_FREQ_HIGH = 14745600; // Support for higher baud rates, standard for modular EX-IO system
   
public:
  // Constructor
   I2CDFPlayer(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t xtal){
    _firstVpin = firstVpin;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    if (xtal == 0){
      _sc16is752_xtal_freq = SC16IS752_XTAL_FREQ_LOW;
    } else { // should be 1
        _sc16is752_xtal_freq = SC16IS752_XTAL_FREQ_HIGH;
      }
    addDevice(this);
   } 
  
public:
  static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t xtal) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) new I2CDFPlayer(firstVpin, nPins, i2cAddress, xtal); 
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
      sendPacket(0x42,0,0);
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
          // added retry counter, sometimes we do not sent keep alive due to other commands sent to DFPlayer
      if (_awaitingResponse && (int32_t)(currentMicros - _timeoutTime) > 0) { // timeout triggered
        if(_retryCounter == 0){ // retry counter out of luck, must take the device to failed state     
          DIAG(F("I2CDFPlayer:%s, DFPlayer not responding on UART channel: 0x%x"), _I2CAddress.toString(), _UART_CH);
          _deviceState = DEVSTATE_FAILED;
          _awaitingResponse = false;
          _playing = false;
          _retryCounter = RETRYCOUNT;
        } else { // timeout and retry protection and recovery of corrupt data frames from DFPlayer
            #ifdef DIAG_I2CDFplayer_playing
              DIAG(F("I2CDFPlayer: %s, DFPlayer timout, retry counter: %d on UART channel: 0x%x"), _I2CAddress.toString(), _retryCounter, _UART_CH);
            #endif
            _timeoutTime = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds// reset timeout
            _awaitingResponse = false; // trigger sending a keep alive 0x42 in processOutgoing()
            _retryCounter --; // decrement retry counter                        
            resetRX_fifo(); // reset the RX fifo as it has corrupt data            
          }
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

 
  // Check for incoming data, and update busy flag and other state accordingly
 
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
            //DIAG(F("I2CDFPlayer: %s, _recvCMD: 0x%x _awaitingResponse: 0x0%x"),_I2CAddress.toString(), _recvCMD, _awaitingResponse);
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
            _retryCounter = RETRYCOUNT; // reset the retry counter as we have received a valid packet
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
      if ( _resetCmd == true){
          sendPacket(0x0C,0,0);
          _resetCmd = false;          
      } else if(_volCmd == true) { // do the volme before palying a track
         if(_requestedVolumeLevel >= 0 && _requestedVolumeLevel <= 30){         
         _currentVolume = _requestedVolumeLevel; // If _requestedVolumeLevel is out of range, sent _currentV1olume      
         }
         sendPacket(0x06, 0x00, _currentVolume);
        _volCmd = false;
      } else if (_playCmd == true) {
        // Change song
        if (_requestedSong != -1) {
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: _requestedVolumeLevel: %u, _requestedSong: %u, _currentFolder: %u _playCmd: 0x%x"), _requestedVolumeLevel, _requestedSong, _currentFolder, _playCmd);
          #endif               
          sendPacket(0x0F, _currentFolder, _requestedSong);  // audio file in folder          
          _requestedSong = -1; 
          _playCmd = false;
        }           
      } //else if (_requestedSong == 0) {
        else if (_stopplayCmd == true) {
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: Stop playing: _stopplayCmd: 0x%x"), _stopplayCmd);
          #endif
        sendPacket(0x16, 0x00, 0x00);  // Stop playing        
        _requestedSong = -1;
        _repeat = false; // reset repeat        
        _stopplayCmd = false;
        } else if (_folderCmd == true) {
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: Folder: _folderCmd: 0x%x, _requestedFolder: %d"), _stopplayCmd, _requestedFolder);
          #endif
          if (_currentFolder != _requestedFolder){
            _currentFolder = _requestedFolder;
          }
          _folderCmd = false;
      } else if (_repeatCmd == true) {
        if(_repeat == false) { // No repeat play currently
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: Repeat: _repeatCmd: 0x%x, _requestedSong: %d, _repeat: 0x0%x"), _repeatCmd, _requestedSong, _repeat);
          #endif 
          sendPacket(0x08, 0x00, _requestedSong);  // repeat playing audio file in root folder          
          _requestedSong = -1;
          _repeat = true; 
        }
        _repeatCmd= false;      
      } else if (_daconCmd == true) { // Always turn DAC on
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: DACON: _daconCmd: 0x%x"), _daconCmd);
        #endif 
        sendPacket(0x1A,0,0x00);
        _daconCmd = false;
      } else if (_eqCmd == true){ // Set Equalizer, values 0x00 - 0x05        
        if (_currentEQvalue != _requestedEQValue){
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: EQ: _eqCmd: 0x%x, _currentEQvalue: 0x0%x, _requestedEQValue: 0x0%x"), _eqCmd, _currentEQvalue, _requestedEQValue);
          #endif 
          _currentEQvalue = _requestedEQValue;
          sendPacket(0x07,0x00,_currentEQvalue);
        }
        _eqCmd = false;
      } else if (_setamCmd == true){ // Set Audio mixer channel
         setGPIO(); // Set the audio mixer channel
         /*        
          if (_audioMixer == 1){ // set to audio mixer 1       
            if (_UART_CH == 0){ 
              TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 0 to high
            } else { // must be UART 1
              TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 1 to high
              }
           //_setamCmd = false;
           //UART_WriteRegister(REG_IOSTATE, TEMP_REG_VAL);
          } else { // set to audio mixer 2
              if (_UART_CH == 0){ 
                TEMP_REG_VAL &= (0x00 << _UART_CH); //Set GPIO pin 0 to Low
              } else { // must be UART 1
                TEMP_REG_VAL &= (0x00 << _UART_CH); //Set GPIO pin 1 to Low
                }
              //_setamCmd = false;
              //UART_WriteRegister(REG_IOSTATE, TEMP_REG_VAL);
            }*/
        _setamCmd = false;        
      } else if ((int32_t)currentMicros - _commandSendTime > 1000000) {
        // Poll device every second that other commands aren't being sent,
        // to check if it's still connected and responding.
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: Send keepalive") );
          #endif
        sendPacket(0x42,0,0); 
        if (!_awaitingResponse) {
          #ifdef DIAG_I2CDFplayer_playing
           DIAG(F("I2CDFPlayer: Send keepalive, _awaitingResponse: 0x0%x"), _awaitingResponse );
          #endif
          _timeoutTime = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds
          _awaitingResponse = true;
        }
      }
    }  
  }


  // Write to a vPin will do nothing
  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
      #ifdef DIAG_IO
        DIAG(F("I2CDFPlayer: Writing to any vPin not supported"));
      #endif
  }


  // WriteAnalogue on first pin uses the nominated value as a file number to start playing, if file number > 0.
  // Volume may be specified as second parameter to writeAnalogue.
  // If value is zero, the player stops playing.  
  // WriteAnalogue on second pin sets the output volume.
  //
  // WriteAnalogue to be done on first vpin
  //
  //void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
  void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t cmd=0) override { 
    if (_deviceState == DEVSTATE_FAILED) return;    
    #ifdef DIAG_IO
      DIAG(F("I2CDFPlayer: VPIN:%u FileNo:%d Volume:%d Command:0x%x"), vpin, value, volume, cmd);
    #endif
    uint8_t pin = vpin - _firstVpin;
    if (pin == 0) { // Enhanced DFPlayer commands, do nothing if not vPin 0     
     // Read command and value
      switch (cmd){
       //case NONE:
       // DFPlayerCmd = cmd;
       // break;
       case DF_PLAY:
        _playCmd = true;
        _volCmd = true;        
        _requestedSong = value;
        _requestedVolumeLevel = volume; 
        _playing = true;        
        break;
        case DF_VOL:
          _volCmd = true;          
          _requestedVolumeLevel = volume;
        break;
       case DF_FOLDER:
        _folderCmd = true;
        if (volume <= 0 || volume > 99){ // Range checking, valid values 1-99, else default to 1
          _requestedFolder = 0x01; // if outside range, default to folder 01  
        } else {
          _requestedFolder = volume;
        }        
        break;
       case DF_REPEATPLAY: // Need to check if _repeat == true, if so do nothing        
        if (_repeat == false) {
           #ifdef DIAG_I2CDFplayer_playing
              DIAG(F("I2CDFPlayer: WriteAnalog Repeat: _repeat: 0x0%x, value: %d _repeatCmd: 0x%x"), _repeat, value, _repeatCmd);
           #endif
          _repeatCmd = true;          
          _requestedSong = value;
          _requestedVolumeLevel = volume;
          _playing = true;         
        }
        break;
       case DF_STOPPLAY:
        _stopplayCmd = true;        
        break;
       case DF_EQ:
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WriteAnalog EQ: cmd: 0x%x, EQ value: 0x%x"), cmd, volume);
        #endif
        _eqCmd = true;        
        if (volume <= 0 || volume > 5) { // If out of range, default to NORMAL
          _requestedEQValue = DF_NORMAL;            
        } else { // Valid EQ parameter range
          _requestedEQValue = volume;     
          }
        break;        
       case DF_RESET:
        _resetCmd = true;      
        break; 
       case DF_DACON: // Works, but without the DACOFF command limited value, except when not relying on DFPlayer default to turn the DAC on
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WrtieAnalog DACON: cmd: 0x%x"), cmd);
        #endif
        _daconCmd = true;
        break;
        case DF_SETAM: // Set the audio mixer channel to 1 or 2
          _setamCmd = true;
          #ifdef DIAG_I2CDFplayer_playing
            DIAG(F("I2CDFPlayer: WrtieAnalog SETAM: cmd: 0x%x"), cmd);
          #endif
          if (volume <= 0 || volume > 2) { // If out of range, default to 1
            _audioMixer = 1;            
          } else { // Valid SETAM parameter in range
              _audioMixer = volume; // _audioMixer valid values 1 or 2
            }          
        break;
       default:
        break;
      }
    }    
  }

  // A read on any pin indicates if the player is still playing.
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return false;
    uint8_t pin = vpin - _firstVpin;
      if (pin == 0) { // Do nothing if not vPin 0
        return _playing;
      }
    }

  void _display() override {
    DIAG(F("I2CDFPlayer Configured on Vpins:%u-%u %S"), _firstVpin, _firstVpin+_nPins-1,
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

  void sendPacket(uint8_t command, uint8_t arg1 = 0, uint8_t arg2 = 0) {
    FIFO_TX_LEVEL = 0; // Reset FIFO_TX_LEVEL    
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
  // IOCONTROL set bit 1 and 2 to 0 indicating that they are GPIO
  // IODIR set all bit to 1 indicating al are output
  // IOSTATE set only bit 0 to 1 for UART 0, or only bit 1 for UART 1  // 
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
    //uint16_t _divisor = (SC16IS752_XTAL_FREQ / PRESCALER) / (BAUD_RATE * 16);
    uint16_t _divisor = (_sc16is752_xtal_freq/PRESCALER)/(BAUD_RATE * 16);  // Calculate _divisor for baudrate
    TEMP_REG_VAL = 0x08; // UART Software reset
    UART_WriteRegister(REG_IOCONTROL, TEMP_REG_VAL);
    TEMP_REG_VAL = 0x00; // Set pins to GPIO mode
    UART_WriteRegister(REG_IOCONTROL, TEMP_REG_VAL);
    TEMP_REG_VAL = 0xFF; //Set all pins as output
    UART_WriteRegister(REG_IODIR, TEMP_REG_VAL);
    UART_ReadRegister(REG_IOSTATE); // Read current state as not to overwrite the other GPIO pins
    TEMP_REG_VAL = _inbuffer[0];
    setGPIO(); // Set the audio mixer channel
    /*
    if (_UART_CH == 0){ // Set Audio mixer channel
      TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 0 to high
    } else { // must be UART 1
      TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 1 to high
    }
    UART_WriteRegister(REG_IOSTATE, TEMP_REG_VAL);
    */
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
  // The RX fifo level is used to check if there are enough bytes to process a frame
  void RX_fifo_lvl(){
    UART_ReadRegister(REG_RXLV);
    FIFO_RX_LEVEL = _inbuffer[0];
    #ifdef DIAG_I2CDFplayer
     if (FIFO_RX_LEVEL > 0){
     //if (FIFO_RX_LEVEL > 0 && FIFO_RX_LEVEL < 10){
      DIAG(F("SC16IS752: At I2C: %s, UART channel: 0x%x, FIFO_RX_LEVEL: 0d%d"), _I2CAddress.toString(), _UART_CH, _inbuffer[0]);
    }
    #endif   
  }

  // When a frame is transmitted from the DFPlayer to the serial port, and at the same time the CS is sending a 42 query
  // the following two frames from the DFPlayer are corrupt. This result in the receive buffer being out of sync and the 
  // CS will complain and generate a timeout.
  // The RX fifo has corrupt data and need to be flushed, this function does that
  // 
  void resetRX_fifo(){
    #ifdef DIAG_I2CDFplayer
      DIAG(F("SC16IS752: At I2C: %s, UART channel: 0x%x, RX fifo reset"), _I2CAddress.toString(), _UART_CH);
    #endif    
    TEMP_REG_VAL = 0x03; // Reset RX fifo
    UART_WriteRegister(REG_FCR, TEMP_REG_VAL);
  }

  // Set or reset GPIO pin 0 and 1 depending on the UART ch
  // This function may be modified in a future release to enable all 8 pins to be set or reset with EX-Rail
  // for various auxilary functions
  void setGPIO(){
    UART_ReadRegister(REG_IOSTATE); // Get the current GPIO pins state from the IOSTATE register
    TEMP_REG_VAL = _inbuffer[0];
    if (_audioMixer == 1){ // set to audio mixer 1
      if (_UART_CH == 0){ 
        TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 0 to high
      } else { // must be UART 1
         TEMP_REG_VAL |= (0x01 << _UART_CH); //Set GPIO pin 1 to high
        }
    } else { // set to audio mixer 2
        if (_UART_CH == 0){ 
          TEMP_REG_VAL &= ~(0x01 << _UART_CH); //Set GPIO pin 0 to Low
        } else { // must be UART 1
           TEMP_REG_VAL &= ~(0x01 << _UART_CH); //Set GPIO pin 1 to Low
          }
      }    
    UART_WriteRegister(REG_IOSTATE, TEMP_REG_VAL);
    _setamCmd = false;  
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
  };

};

#endif // IO_I2CDFPlayer_h
