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
 * 2023, Added NXP SC16IS752 I2C Dual UART to enable the DFPlayer connection over the I2C bus
 * The SC16IS752 has 64 bytes TX & RX FIFO buffer
 * First version without interrupts from I2C UART and only RX/TX are used, interrupts may not be
 * needed as the RX Fifo holds the reply
 * 
 * 2024, Issue with using both UARTs simultaniously, the first configured in myHal.cpp seems to get 
 * overwritten by the second configured.
 * Possible solution is to handle both uarts in the same IO_I2CDFPLayer.h file as it seems that I2CManager
 * is not able to disinguise the 2 UARTs as they have the same I2C address
 * 
 * 2024-01-22: Taken sendpacket and associated code out of _begin(), let _loop() handle it, need testing
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
#define DIAG_I2CDFplayer
//#define DIAG_I2CDFplayer_data
//#define DIAG_I2CDFplayer_reg
//#define DIAG_I2CDFplayer_playing

class I2CDFPlayer : public IODevice {

private: 
// Common parameters
  const uint8_t MAXVOLUME=30;
  // Communication parameters for the DFPlayer are fixed at 8 bit, No parity, 1 stopbit
  uint8_t WORD_LEN = 0x03;    // Value LCR bit 0,1
  uint8_t STOP_BIT = 0x00;    // Value LCR bit 2 
  uint8_t PARITY_ENA = 0x00;  // Value LCR bit 3
  uint8_t PARITY_TYPE = 0x00; // Value LCR bit 4
  uint32_t BAUD_RATE = 9600;
  uint8_t PRESCALER = 0x01;   // Value MCR bit 7
  uint8_t RETRYCOUNT_INIT = 0x3;
  unsigned long _sc16is752_xtal_freq;
  unsigned long SC16IS752_XTAL_FREQ_LOW = 1843200; // To support cheap eBay/AliExpress SC16IS752 boards
  unsigned long SC16IS752_XTAL_FREQ_HIGH = 14745600; // Support for higher baud rates, standard for modular EX-IO system  
  uint8_t TEMP_REG_VAL = 0x00;
  uint8_t _inputIndex = 0;
  uint8_t _recvCMD;  // Last received command code byte 
//  uint8_t _inbuffer[10]; // common buffer for processing data from DFPLayer
  uint8_t RX_BUFFER = 0x00; // nr of bytes copied into _inbuffer
  uint8_t status; // I2C status
  uint8_t _uartSent = _UART_1; // Last uart used to send packet
  uint8_t _uartReceive = _UART_1; // Last uart to receive packet
  uint8_t _outbuffer[11]; // DFPlayer command is 10 bytes + 1 byte register address & UART channel
  uint8_t _inbuffer[10]; // expected DFPlayer return 10 bytes

  
  // SC16IS752 defines
  I2CAddress _I2CAddress;
  I2CRB _rb;

  
// Parameters and variables for UART 0
  uint8_t FIFO_RX_LEVEL_0 = 0x00;
  uint8_t FIFO_TX_LEVEL_0 = 0x00;
  uint8_t _UART_0 = 0x00; 
  uint8_t _retryCounter_0 = RETRYCOUNT_INIT; // Max retries before timing out
  bool _playing_0 = false;    
  uint8_t _deviceState_0 = DEVSTATE_NORMAL; // Devivce State for DFPLayer
  unsigned long _commandSendTime_0; // Time (us) that last transmit took place.
  unsigned long _timeoutTime_0;
  bool _awaitingResponse_0 = false;    
  uint8_t _requestedVolumeLevel_0 = MAXVOLUME;
  uint8_t _currentVolume_0 = MAXVOLUME;
  int _requestedSong_0 = -1;  // -1=none, 0=stop, >0=file number
  bool _repeat_0 = false; // audio file is repeat playing
  uint8_t _previousCmd_0 = true;
  bool _playCmd_0 = false;
  bool _volCmd_0 = false;
  bool _folderCmd_0 = false;
  uint8_t _requestedFolder_0 = 0x01; // default to folder 01
  uint8_t _currentFolder_0 = 0x01; // default to folder 01
  bool _repeatCmd_0 = false;
  bool _stopplayCmd_0 = false;
  bool _resetCmd_0 = false;
  bool _eqCmd_0 = false;
  uint8_t _requestedEQValue_0 = NORMAL;
  uint8_t _currentEQvalue_0 = NORMAL; // start equalizer value
  bool _daconCmd_0 = false;
  uint8_t _audioMixer_0 = 0x01; // Default to output amplifier 1
  bool _setamCmd_0 = false; // Set the Audio mixer channel
 

// Parameters and variables for UART 1
  uint8_t FIFO_RX_LEVEL_1 = 0x00;
  uint8_t FIFO_TX_LEVEL_1 = 0x00;
  uint8_t _UART_1 = 0x01; 
  uint8_t _retryCounter_1 = RETRYCOUNT_INIT; // Max retries before timing out
  bool _playing_1 = false;    
  uint8_t _deviceState_1 = DEVSTATE_NORMAL; // Devivce State for DFPLayer
  unsigned long _commandSendTime_1; // Time (us) that last transmit took place.
  unsigned long _timeoutTime_1;
  bool _awaitingResponse_1 = false;    
  uint8_t _requestedVolumeLevel_1 = MAXVOLUME;
  uint8_t _currentVolume_1 = MAXVOLUME;
  int _requestedSong_1 = -1;  // -1=none, 0=stop, >0=file number
  bool _repeat_1 = false; // audio file is repeat playing
  uint8_t _previousCmd_1 = true;
  bool _playCmd_1 = false;
  bool _volCmd_1 = false;
  bool _folderCmd_1 = false;
  uint8_t _requestedFolder_1 = 0x01; // default to folder 01
  uint8_t _currentFolder_1 = 0x01; // default to folder 01
  bool _repeatCmd_1 = false;
  bool _stopplayCmd_1 = false;
  bool _resetCmd_1 = false;
  bool _eqCmd_1 = false;
  uint8_t _requestedEQValue_1 = NORMAL;
  uint8_t _currentEQvalue_1 = NORMAL; // start equalizer value
  bool _daconCmd_1 = false;
  uint8_t _audioMixer_1 = 0x01; // Default to output amplifier 1
  bool _setamCmd_1 = false; // Set the Audio mixer channel

   
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
      Init_SC16IS752(_UART_0); // Initialize _UART_0  
      //Init_SC16IS752(_UART_1); // Initialize _UART_1          

      // /*
      if (_deviceState_0 == DEVSTATE_FAILED || _deviceState_1 == DEVSTATE_FAILED){
        DIAG(F("SC16IS752 I2C:%s initialization failed"), _I2CAddress.toString());
        } 
      } else {
          DIAG(F("SC16IS752 I2C:%s UART not detected"), _I2CAddress.toString());
        } // */
      
      #if defined(DIAG_IO)
        _display();
      #endif
      // Now init DFPlayer 0
      // Send a query to the device to see if it responds
      _deviceState_0 = DEVSTATE_INITIALISING; // _deviceState is for uart 0      
      // if (status == I2C_STATUS_OK){
         //sendPacket(0x42,0,0,_UART_0); // take this out of _begin() and let _loop handle it
         //_timeoutTime_0 = micros() + 5000000UL;  // 5 second timeout      
         //_awaitingResponse_0 = true;         
      //  }

    if (I2CManager.exists(_I2CAddress)){
      DIAG(F("SC16IS752 I2C:%s UART detected"), _I2CAddress.toString());
      Init_SC16IS752(_UART_1); // Initialize UART_1      
      if (_deviceState_1 == DEVSTATE_FAILED){
        DIAG(F("SC16IS752 I2C:%s initialization failed, UART: %d"), _I2CAddress.toString(), _UART_1);
        } 
    } else {
        DIAG(F("SC16IS752 I2C:%s UART not detected, UART: %d"), _I2CAddress.toString(), _UART_0);
      }
    #if defined(DIAG_IO)
       _display();
    #endif
    // Now init DFPlayer 1
    // Send a query to the device to see if it responds
    _deviceState_1 = DEVSTATE_INITIALISING; // _deviceState is for UART 1
    //if (status == I2C_STATUS_OK){
       // sendPacket(0x42,0,0,_UART_1); // take this out of _begin() and let _loop handle it
       //_timeoutTime_1 = micros() + 5000000UL;  // 5 second timeout      
       //_awaitingResponse_1 = true; 
    //  }
  }
  
  
  void _loop(unsigned long currentMicros) override {
    // Read responses from device  
    uint8_t _uart;  
    if(_rb.isBusy()) {
     DIAG(F("I2CDFPlayer: %s, Loop, _rb.isBusy, do nothing"), _I2CAddress.toString());
     return;    
    }
    status = _rb.status;        
    if (status == I2C_STATUS_OK) {      
      _uart = _UART_0;
      processIncoming(currentMicros, _uart);
          // Check if a command sent to device has timed out.  Allow 0.5 second for response
          // added retry counter, sometimes we do not sent keep alive due to other commands sent to DFPlayer
      if(_uart == _UART_0){
        if (_awaitingResponse_0 && (int32_t)(currentMicros - _timeoutTime_0) > 0) { // timeout triggered
          if(_retryCounter_0 == 0){ // retry counter out of luck, must take the device to failed state     
            DIAG(F("I2CDFPlayer:%s, DFPlayer not responding on UART: %d"), _I2CAddress.toString(), _uart);
            _deviceState_0 = DEVSTATE_FAILED; // Fail uart 0
            _awaitingResponse_0 = false;
            _playing_0 = false;
            _retryCounter_0 = RETRYCOUNT_INIT;
          } else { // timeout and retry protection and recovery of corrupt data frames from DFPlayer
              #ifdef DIAG_I2CDFplayer_playing
                DIAG(F("I2CDFPlayer: %s, DFPlayer timout, retry counter_0: %d on UART: %d"), _I2CAddress.toString(), _retryCounter_0, _uart);
              #endif
              _timeoutTime_0 = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds// reset timeout
              _awaitingResponse_0 = false; // trigger sending a keep alive 0x42 in processOutgoing()
              _retryCounter_0 --; // decrement retry counter                        
              resetRX_fifo(_uart); // reset the RX fifo as it has corrupt data            
            }
        }
      }
      _uart = _UART_1;      
      
      //processIncoming(currentMicros, _uart);
      
          // Check if a command sent to device has timed out.  Allow 0.5 second for response
          // added retry counter, sometimes we do not sent keep alive due to other commands sent to DFPlayer
      if(_uart == _UART_1){
        if (_awaitingResponse_1 && (int32_t)(currentMicros - _timeoutTime_1) > 0) { // timeout triggered
          if(_retryCounter_1 == 0){ // retry counter out of luck, must take the device to failed state     
            DIAG(F("I2CDFPlayer:%s, DFPlayer not responding on UART: %d"), _I2CAddress.toString(), _uart);
            _deviceState_1 = DEVSTATE_FAILED; // Fail uart 1
            _awaitingResponse_1 = false;
            _playing_1 = false;
            _retryCounter_1 = RETRYCOUNT_INIT;
          } else { // timeout and retry protection and recovery of corrupt data frames from DFPlayer
              #ifdef DIAG_I2CDFplayer_playing
                DIAG(F("I2CDFPlayer: %s, DFPlayer timout, retry counter_1: %d on UART: %d"), _I2CAddress.toString(), _retryCounter_1, _uart);
              #endif
              _timeoutTime_1 = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds// reset timeout
              _awaitingResponse_1 = false; // trigger sending a keep alive 0x42 in processOutgoing()
              _retryCounter_1 --; // decrement retry counter                        
              resetRX_fifo(_uart); // reset the RX fifo as it has corrupt data            
            }
        }
      }
    }

    if(_rb.isBusy()) {
      #ifdef DIAG_I2CDFplayer_playing
        DIAG(F("I2CDFPlayer: %s, _rb.isBusy"), _I2CAddress.toString());
      #endif
      return; // I2C operation still ongoing return       
    }
    status = _rb.status;  
    if (status == I2C_STATUS_OK) {
      if(!_rb.isBusy()){ // not busy, save to sent data 
        if (_uartSent == _UART_1){ // Previous sent was _UART_1, now sent _UART_0
            //#ifdef DIAG_I2CDFplayer_playing
            //  DIAG(F("I2CDFPlayer: %s, sent uart 0"), _I2CAddress.toString());
            //#endif
            // Send any commands that need to go.
 
            processOutgoing(currentMicros, _UART_0);
           
            _uartSent = _UART_0;
          } else {
            //#ifdef DIAG_I2CDFplayer_playing
            //  DIAG(F("I2CDFPlayer: %s, sent uart 1"), _I2CAddress.toString());
            //#endif
            
            //processOutgoing(currentMicros, _UART_1);
            
            _uartSent = _UART_1;
          }
        delayUntil(currentMicros + 10000); // Only enter every 10ms    
      }
    }
  }

 
  // Check for incoming data, and update busy flag and other state accordingly
 
  void processIncoming(unsigned long currentMicros, uint8_t _uart) {
    // Expected message is in the form "7E FF 06 3D xx xx xx xx xx EF"
    RX_fifo_lvl(_uart);
    if(_uart == _UART_0){ // Process uart 0
      if (FIFO_RX_LEVEL_0 >= 10) {      
        #ifdef DIAG_I2CDFplayer
          DIAG(F("I2CDFPlayer: %s Retrieving data from RX Fifo on UART: %d, FIFO_RX_LEVEL_0: %d"),_I2CAddress.toString(), _uart, FIFO_RX_LEVEL_0); 
        #endif
        _outbuffer[0] = REG_RHR << 3 | _uart << 1;
        // Only copy 10 bytes from RX FIFO, there maybe additional partial return data after a track is finished playing in the RX FIFO
        I2CManager.read(_I2CAddress, _inbuffer, 10, _outbuffer, 1); // inbuffer[] has the data now
        RX_BUFFER = 10; // We have copied 10 bytes from RX FIFO to _inbuffer
         #ifdef DIAG_I2CDFplayer_data
            DIAG(F("SC16IS752: I2C: %s, Receive data, RX FIFO Data, UART: %d"), _I2CAddress.toString(), _uart);
            for (int i = 0; i < sizeof _inbuffer; i++){
              DIAG(F("SC16IS752: Data _inbuffer[0x%x]: 0x%x"), i, _inbuffer[i]);  
            }
         #endif
      } else { //RX fifo level less < 10, do nothing
          FIFO_RX_LEVEL_0 = 0; //set to 0, we'll read a fresh FIFO_RX_LEVEL_0 next time
          return; // No data or not enough data in rx fifo, check again next time around
        } 
    } else { // Process uart 1
       if (FIFO_RX_LEVEL_1 >= 10) {      
        #ifdef DIAG_I2CDFplayer
          DIAG(F("I2CDFPlayer: %s Retrieving data from RX Fifo on UART: %d, FIFO_RX_LEVEL_1: %d"),_I2CAddress.toString(), _uart, FIFO_RX_LEVEL_1); 
        #endif
        _outbuffer[0] = REG_RHR << 3 | _uart << 1;
        // Only copy 10 bytes from RX FIFO, there maybe additional partial return data after a track is finished playing in the RX FIFO
        I2CManager.read(_I2CAddress, _inbuffer, 10, _outbuffer, 1); // inbuffer_1[] has the data now
        RX_BUFFER = 10; // We have copied 10 bytes from RX FIFO to _inbuffer
         #ifdef DIAG_I2CDFplayer_data
            DIAG(F("SC16IS752: I2C: %s, Receive data, RX FIFO Data, UART: %d"), _I2CAddress.toString(), _uart);
            for (int i = 0; i < sizeof _inbuffer; i++){
              DIAG(F("SC16IS752: Data _inbuffer[0x%x]: 0x%x"), i, _inbuffer[i]);  
            }
         #endif
      } else { //RX fifo level less < 10, do nothing
          FIFO_RX_LEVEL_1 = 0; //set to 0, we'll read a fresh FIFO_RX_LEVEL_1 next time
          return; // No data or not enough data in rx fifo, check again next time around
        } 
    }

    // Tranfer _inbuffer or _inbuffer to _inbuffer (common buffer to process incomming frame from DFPLayer)
    // _inbuffer is used for both uarts
    if (_uart == _UART_0){
      for( int i = 0;i < sizeof _inbuffer; i++){
        _inbuffer[i] = _inbuffer[i];
      }      
    } else if (_uart == _UART_1){
      for( int i = 0;i < sizeof _inbuffer; i++){
        _inbuffer[i] = _inbuffer[i];
      }
    }

    bool ok = false;    
    while (RX_BUFFER != 0) { 
      int c = _inbuffer[_inputIndex]; // Start at 0, increment to FIFO_RX_LEVEL
      switch (_inputIndex) {
        case 0:
          if (c == 0x7E) ok = true; // Start flag
          break;
        case 1:
          if (c == 0xFF) ok = true; // Version
          break;
        case 2:
          if (c== 0x06) ok = true; // Length
          break;
        case 3:
          _recvCMD = c; // CMD byte
          ok = true;
          break;
        case 6:
          switch (_recvCMD) {
            #ifdef DIAG_I2CDFplayer_data
              if(_uart == _UART_0){
                DIAG(F("I2CDFPlayer: %s, uart: %d, _recvCMD: 0x%x _awaitingResponse_0: 0x0%x"),_I2CAddress.toString(), _uart, _recvCMD, _awaitingResponse_0);
              } else { // uart 1
                 DIAG(F("I2CDFPlayer: %s, uart: %d, _recvCMD: 0x%x _awaitingResponse_1: 0x0%x"),_I2CAddress.toString(), _uart, _recvCMD, _awaitingResponse_1);
                }
            #endif
            case 0x42:
              // Response to status query
              if(_uart == _UART_0){
                _playing_0 = (c != 0); // Mark the DFPLayer online and cancel timeout                
                if (_deviceState_0 == DEVSTATE_INITIALISING) {
                  _deviceState_0 = DEVSTATE_NORMAL;
                  #ifdef DIAG_I2CDFplayer_playing
                    DIAG(F("I2CDFPlayer: %s, keepalive response: 0x%x, uart: %d, _deviceState_0: 0x0%x"),_I2CAddress.toString(), _recvCMD, _uart, _deviceState_0);
                  #endif 
                  #ifdef DIAG_IO
                  _display();
                  #endif
                }
                #ifdef DIAG_I2CDFplayer_playing
                  DIAG(F("I2CDFPlayer: %s, keepalive response: 0x%x, uart: %d"), _I2CAddress.toString(), _recvCMD, _uart);
                #endif 
                _awaitingResponse_0 = false;                
              } else { // uart 1
                  _playing_1 = (c != 0); // Mark the DFPLayer online and cancel timeout                  
                  if (_deviceState_1 == DEVSTATE_INITIALISING) {
                    _deviceState_1 = DEVSTATE_NORMAL;
                    #ifdef DIAG_I2CDFplayer_playing
                      DIAG(F("I2CDFPlayer: %s, keepalive response: 0x%x, uart: %d, _deviceState_1: 0x0%x"),_I2CAddress.toString(), _recvCMD, _uart, _deviceState_1);
                    #endif 
                    #ifdef DIAG_IO
                      _display();
                    #endif
                  }
                  #ifdef DIAG_I2CDFplayer_playing
                    DIAG(F("I2CDFPlayer: %s, keepalive response: 0x%x, uart: %d"), _I2CAddress.toString(), _recvCMD, _uart);
                  #endif 
                  _awaitingResponse_1 = false;                
                }              
              break;
            case 0x3d:            
              // End of play
              if (_uart == _UART_0){
                if (_playing_0) {
                  #ifdef DIAG_IO
                    DIAG(F("I2CDFPlayer: Finished, uart: %d"), _uart);
                  #endif
                  _playing_0 = false;
                }
              } else { // uart 1 finished playing
                  if (_playing_1) {
                  #ifdef DIAG_IO
                    DIAG(F("I2CDFPlayer: Finished, uart: %d"), _uart);
                  #endif
                  _playing_1 = false;
                }
              }              
              break;
            case 0x40:
              // Error codes; 1: Module Busy
               if (_uart == _UART_0){
                  DIAG(F("I2CDFPlayer: Error %d returned from device, uart: %d"), c, _uart);
                  _playing_0 = false;
               } else { // uart 1 error message
                   DIAG(F("I2CDFPlayer: Error %d returned from device, uart: %d"), c, _uart);
                  _playing_1 = false;
                 }
              break;
          }
          ok = true;
          break;
        case 4: case 5: case 7: case 8: 
          ok = true;  // Skip over these bytes in message.
          break;
        case 9:
          if (c==0xef) { // Message finished
            if(_uart == _UART_0){
              _retryCounter_0 = RETRYCOUNT_INIT; // reset the retry counter as we have received a valid packet
            } else { //uart 1
                _retryCounter_1 = RETRYCOUNT_INIT; // reset the retry counter as we have received a valid packet
              }
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
  void processOutgoing(unsigned long currentMicros, uint8_t _uart) {
    // When two commands are sent in quick succession, the device will often fail to 
    // execute one.  Testing has indicated that a delay of 100ms or more is required
    // between successive commands to get reliable operation.
    // If 100ms has elapsed since the last thing sent, then check if there's some output to do.

    if(_uart == _UART_0){
      if (((int32_t)currentMicros - _commandSendTime_0) > 100000) {
        if ( _resetCmd_0 == true){
            sendPacket(0x0C,0,0, _uart);
            _resetCmd_0 = false;          
        } else if(_volCmd_0 == true) { // do the volme before palying a track
              if(_requestedVolumeLevel_0 >= 0 && _requestedVolumeLevel_0 <= 30){         
              _currentVolume_0 = _requestedVolumeLevel_0; // If _requestedVolumeLevel is out of range, sent _currentV1olume      
            }
            sendPacket(0x06, 0x00, _currentVolume_0, _uart);
            _volCmd_0 = false;
          } else if (_playCmd_0 == true) {
              // Change song
              if (_requestedSong_0 != -1) {
                #ifdef DIAG_I2CDFplayer_playing
                  DIAG(F("I2CDFPlayer: _requestedVolumeLevel_0: %u, _requestedSong_0: %u, _currentFolder_0: %u _playCmd_0: 0x%x"), _requestedVolumeLevel_0, _requestedSong_0, _currentFolder_0, _playCmd_0);
                #endif               
                sendPacket(0x0F, _currentFolder_0, _requestedSong_0, _uart);  // audio file in folder          
                _requestedSong_0 = -1; 
                _playCmd_0 = false;
              }           
            } //else if (_requestedSong_0 == 0) {
              else if (_stopplayCmd_0 == true) {
                #ifdef DIAG_I2CDFplayer_playing
                  DIAG(F("I2CDFPlayer: Stop playing: _stopplayCmd_0: 0x%x"), _stopplayCmd_0);
                #endif
                sendPacket(0x16, 0x00, 0x00, _uart);  // Stop playing        
                _requestedSong_0 = -1;
                _repeat_0 = false; // reset repeat for uart 0        
                 _stopplayCmd_0 = false;
              } else if (_folderCmd_0 == true) {
                  #ifdef DIAG_I2CDFplayer_playing
                    DIAG(F("I2CDFPlayer: Folder: _folderCmd_0: 0x%x, _requestedFolder_0: %d"), _stopplayCmd_0, _requestedFolder_0);
                  #endif
                  if (_currentFolder_0 != _requestedFolder_0){
                    _currentFolder_0 = _requestedFolder_0;
                  }
                  _folderCmd_0 = false;
                } else if (_repeatCmd_0 == true) {
                    if(_repeat_0 == false) { // No repeat play currently
                      #ifdef DIAG_I2CDFplayer_playing
                        DIAG(F("I2CDFPlayer: Repeat: _repeatCmd_0: 0x%x, _requestedSong_0: %d, _repeat_0: 0x0%x"), _repeatCmd_0, _requestedSong_0, _repeat_0);
                      #endif 
                      sendPacket(0x08, 0x00, _requestedSong_0, _uart);  // repeat playing audio file in root folder          
                      _requestedSong_0 = -1;
                      _repeat_0 = true; 
                    }
                    _repeatCmd_0= false;      
                  } else if (_daconCmd_0 == true) { // Always turn DAC on
                      #ifdef DIAG_I2CDFplayer_playing
                        DIAG(F("I2CDFPlayer: DACON: _daconCmd_0: 0x%x"), _daconCmd_0);
                      #endif 
                      sendPacket(0x1A,0,0x00, _uart);
                      _daconCmd_0 = false;
                    } else if (_eqCmd_0 == true){ // Set Equalizer, values 0x00 - 0x05        
                        if (_currentEQvalue_0 != _requestedEQValue_0){
                          #ifdef DIAG_I2CDFplayer_playing
                            DIAG(F("I2CDFPlayer: EQ: _eqCmd_0: 0x%x, _currentEQvalue_0: 0x0%x, _requestedEQValue_0: 0x0%x"), _eqCmd_0, _currentEQvalue_0, _requestedEQValue_0);
                          #endif 
                          _currentEQvalue_0 = _requestedEQValue_0;
                          sendPacket(0x07,0x00,_currentEQvalue_0, _uart);
                        }
                        _eqCmd_0 = false;
                      } else if (_setamCmd_0 == true){ // Set Audio mixer channel
                          setGPIO(_uart); // Set the audio mixer channel
                          _setamCmd_0 = false;        
                        } else if ((int32_t)currentMicros - _commandSendTime_0 > 1000000) {
                            // Poll device every second that other commands aren't being sent,
                            // to check if it's still connected and responding.
                            #ifdef DIAG_I2CDFplayer_playing
                              DIAG(F("I2CDFPlayer: Send keepalive, uart: %d") , _uart);
                            #endif
                            sendPacket(0x42,0,0, _uart); 
                            if (!_awaitingResponse_0) {
                            #ifdef DIAG_I2CDFplayer_playing
                              DIAG(F("I2CDFPlayer: Send keepalive, _awaitingResponse_0: 0x0%x, , uart: %d"), _awaitingResponse_0, _uart);
                            #endif
                            _timeoutTime_0 = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds
                            _awaitingResponse_0 = true;
                            }
                          }
      }
     } else if(_uart == _UART_1){
      if (((int32_t)currentMicros - _commandSendTime_1) > 100000) {
        if ( _resetCmd_1 == true){
            sendPacket(0x0C,0,0, _uart);
            _resetCmd_1 = false;          
        } else if(_volCmd_1 == true) { // do the volme before palying a track
              if(_requestedVolumeLevel_1 >= 0 && _requestedVolumeLevel_1 <= 30){         
              _currentVolume_1 = _requestedVolumeLevel_1; // If _requestedVolumeLevel is out of range, sent _currentV1olume      
            }
            sendPacket(0x06, 0x00, _currentVolume_1, _uart);
            _volCmd_1 = false;
          } else if (_playCmd_1 == true) {
              // Change song
              if (_requestedSong_1 != -1) {
                #ifdef DIAG_I2CDFplayer_playing
                  DIAG(F("I2CDFPlayer: _requestedVolumeLevel_1: %u, _requestedSong_1: %u, _currentFolder_1: %u _playCmd_1: 0x%x"), _requestedVolumeLevel_1, _requestedSong_1, _currentFolder_1, _playCmd_1);
                #endif               
                sendPacket(0x0F, _currentFolder_1, _requestedSong_1, _uart);  // audio file in folder          
                _requestedSong_1 = -1; 
                _playCmd_1 = false;
              }           
            } //else if (_requestedSong_0 == 0) {
              else if (_stopplayCmd_1 == true) {
                #ifdef DIAG_I2CDFplayer_playing
                  DIAG(F("I2CDFPlayer: Stop playing: _stopplayCmd_1: 0x%x"), _stopplayCmd_1);
                #endif
                sendPacket(0x16, 0x00, 0x00, _uart);  // Stop playing        
                _requestedSong_1 = -1;
                _repeat_1 = false; // reset repeat for uart 0        
                 _stopplayCmd_1 = false;
              } else if (_folderCmd_1 == true) {
                  #ifdef DIAG_I2CDFplayer_playing
                    DIAG(F("I2CDFPlayer: Folder: _folderCmd_1: 0x%x, _requestedFolder_1: %d"), _stopplayCmd_1, _requestedFolder_1);
                  #endif
                  if (_currentFolder_1 != _requestedFolder_1){
                    _currentFolder_1 = _requestedFolder_1;
                  }
                  _folderCmd_1 = false;
                } else if (_repeatCmd_1 == true) {
                    if(_repeat_1 == false) { // No repeat play currently
                      #ifdef DIAG_I2CDFplayer_playing
                        DIAG(F("I2CDFPlayer: Repeat: _repeatCmd_1: 0x%x, _requestedSong_1: %d, _repeat_1: 0x0%x"), _repeatCmd_1, _requestedSong_1, _repeat_1);
                      #endif 
                      sendPacket(0x08, 0x00, _requestedSong_0, _uart);  // repeat playing audio file in root folder          
                      _requestedSong_1 = -1;
                      _repeat_1 = true; 
                    }
                    _repeatCmd_1= false;      
                  } else if (_daconCmd_1 == true) { // Always turn DAC on
                      #ifdef DIAG_I2CDFplayer_playing
                        DIAG(F("I2CDFPlayer: DACON: _daconCmd_1: 0x%x"), _daconCmd_1);
                      #endif 
                      sendPacket(0x1A,0,0x00, _uart);
                      _daconCmd_1 = false;
                    } else if (_eqCmd_1 == true){ // Set Equalizer, values 0x00 - 0x05        
                        if (_currentEQvalue_1 != _requestedEQValue_1){
                          #ifdef DIAG_I2CDFplayer_playing
                            DIAG(F("I2CDFPlayer: EQ: _eqCmd_1: 0x%x, _currentEQvalue_1: 0x0%x, _requestedEQValue_1: 0x0%x"), _eqCmd_1, _currentEQvalue_1, _requestedEQValue_1);
                          #endif 
                          _currentEQvalue_1 = _requestedEQValue_1;
                          sendPacket(0x07,0x00,_currentEQvalue_0, _uart);
                        }
                        _eqCmd_1 = false;
                      } else if (_setamCmd_1 == true){ // Set Audio mixer channel
                          setGPIO(_uart); // Set the audio mixer channel
                          _setamCmd_1 = false;        
                        } else if ((int32_t)currentMicros - _commandSendTime_1 > 1000000) {
                            // Poll device every second that other commands aren't being sent,
                            // to check if it's still connected and responding.
                            #ifdef DIAG_I2CDFplayer_playing
                              DIAG(F("I2CDFPlayer: Send keepalive, uart: %d") , _uart);
                            #endif
                            sendPacket(0x42,0,0, _uart); 
                            if (!_awaitingResponse_1) {
                            #ifdef DIAG_I2CDFplayer_playing
                              DIAG(F("I2CDFPlayer: Send keepalive, _awaitingResponse_1: 0x0%x, , uart: %d"), _awaitingResponse_1, _uart);
                            #endif
                            _timeoutTime_1 = currentMicros + 5000000UL;  // Timeout if no response within 5 seconds
                            _awaitingResponse_1 = true;
                            }
                          }
      }
   }
  }


  // Write to a vPin will do nothing
  void _write(VPIN vpin, int value) override {
    if (_deviceState_0 == DEVSTATE_FAILED) return;
    if (_deviceState_1 == DEVSTATE_FAILED) return;
      #ifdef DIAG_IO
        DIAG(F("I2CDFPlayer: Writing to any vPin not supported"));
      #endif
  }


  // WriteAnalogue on first pin uses the nominated value as a file number to start playing, if file number > 0.
  // Volume may be specified as second parameter to writeAnalogue.
  // If value is zero, the player stops playing.  
  // WriteAnalogue on second pin sets the output volume.
  //
  //
  //void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t=0) override { 
  void _writeAnalogue(VPIN vpin, int value, uint8_t volume=0, uint16_t cmd=0) override { 
    if (_deviceState_0 == DEVSTATE_FAILED) return;    
    if (_deviceState_1 == DEVSTATE_FAILED) return;
    #ifdef DIAG_IO
      DIAG(F("I2CDFPlayer: VPIN:%u FileNo:%d Volume:%d Command:0x%x"), vpin, value, volume, cmd);
    #endif
    uint8_t pin = vpin - _firstVpin;
    if (pin == 0) { // Enhanced DFPlayer commands, vPin 0 for uart 0     
     // Read command and value
      switch (cmd){
       case PLAY:
        _playCmd_0 = true;
        _volCmd_0 = true;        
        _requestedSong_0 = value;
        _requestedVolumeLevel_0 = volume; 
        _playing_0 = true;        
        break;
        case VOL:
          _volCmd_0 = true;          
          _requestedVolumeLevel_0 = volume;
        break;
       case FOLDER:
        _folderCmd_0 = true;
        if (volume <= 0 || volume > 99){ // Range checking, valid values 1-99, else default to 1
          _requestedFolder_0 = 0x01; // if outside range, default to folder 01  
        } else {
          _requestedFolder_0 = volume;
        }        
        break;
       case REPEATPLAY: // Need to check if _repeat == true, if so do nothing        
        if (_repeat_0 == false) {
           #ifdef DIAG_I2CDFplayer_playing
              DIAG(F("I2CDFPlayer: WriteAnalog Repeat: _repeat_0: 0x0%x, value: %d _repeatCmd_0: 0x%x"), _repeat_0, value, _repeatCmd_0);
           #endif
          _repeatCmd_0 = true;          
          _requestedSong_0 = value;
          _requestedVolumeLevel_0 = volume;
          _playing_0 = true;         
        }
        break;
       case STOPPLAY:
        _stopplayCmd_0 = true;        
        break;
       case EQ:
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WriteAnalog EQ: cmd: 0x%x, EQ value: 0x%x"), cmd, volume);
        #endif
        _eqCmd_0 = true;        
        if (volume <= 0 || volume > 5) { // If out of range, default to NORMAL
          _requestedEQValue_0 = NORMAL;            
        } else { // Valid EQ parameter range
          _requestedEQValue_0 = volume;     
          }
        break;        
       case RESET: // Reset the DFPlayer module
        _resetCmd_0 = true;      
        break; 
       case DACON: // Works, but without the DACOFF command limited value, except when not relying on DFPlayer default to turn the DAC on
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WrtieAnalog DACON: cmd: 0x%x"), cmd);
        #endif
        _daconCmd_0 = true;
        break;
        case SETAM: // Set the audio mixer channel to 1 or 2
          _setamCmd_0 = true;
          #ifdef DIAG_I2CDFplayer_playing
            DIAG(F("I2CDFPlayer: WrtieAnalog SETAM: value: %d, cmd: 0x%x"), value, cmd);
          #endif
          if (volume <= 0 || volume > 2) { // If out of range, default to 1
            _audioMixer_0 = 1;            
          } else { // Valid SETAM parameter in range
              _audioMixer_0 = volume; // _audioMixer_0 valid values 1 or 2
            }          
        break;
       default:
        break;
      }
    } else if(pin == 1){ // Enhanced DFPlayer commands, vPin 0 for uart 0
       switch (cmd){
       case PLAY:
        _playCmd_1 = true;
        _volCmd_1 = true;        
        _requestedSong_1 = value;
        _requestedVolumeLevel_1 = volume; 
        _playing_1 = true;        
        break;
        case VOL:
          _volCmd_1 = true;          
          _requestedVolumeLevel_1 = volume;
        break;
       case FOLDER:
        _folderCmd_1 = true;
        if (volume <= 0 || volume > 99){ // Range checking, valid values 1-99, else default to 1
          _requestedFolder_1 = 0x01; // if outside range, default to folder 01  
        } else {
          _requestedFolder_1 = volume;
        }        
        break;
       case REPEATPLAY: // Need to check if _repeat == true, if so do nothing        
        if (_repeat_1 == false) {
           #ifdef DIAG_I2CDFplayer_playing
              DIAG(F("I2CDFPlayer: WriteAnalog Repeat: _repeat_1: 0x0%x, value: %d _repeatCmd_1: 0x%x"), _repeat_1, value, _repeatCmd_1);
           #endif
          _repeatCmd_1 = true;          
          _requestedSong_1 = value;
          _requestedVolumeLevel_1 = volume;
          _playing_1 = true;         
        }
        break;
       case STOPPLAY:
        _stopplayCmd_1 = true;        
        break;
       case EQ:
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WriteAnalog EQ: cmd: 0x%x, EQ value: 0x%x"), cmd, volume);
        #endif
        _eqCmd_1 = true;        
        if (volume <= 0 || volume > 5) { // If out of range, default to NORMAL
          _requestedEQValue_1 = NORMAL;            
        } else { // Valid EQ parameter range
          _requestedEQValue_1 = volume;     
          }
        break;        
       case RESET: // Reset the DFPlayer module
        _resetCmd_1 = true;      
        break; 
       case DACON: // Works, but without the DACOFF command limited value, except when not relying on DFPlayer default to turn the DAC on
        #ifdef DIAG_I2CDFplayer_playing
          DIAG(F("I2CDFPlayer: WrtieAnalog DACON: cmd: 0x%x"), cmd);
        #endif
        _daconCmd_1 = true;
        break;
        case SETAM: // Set the audio mixer channel to 1 or 2
          _setamCmd_1 = true;
          #ifdef DIAG_I2CDFplayer_playing
            DIAG(F("I2CDFPlayer: WrtieAnalog SETAM: value: %d, cmd: 0x%x"), value, cmd);
          #endif
          if (volume <= 0 || volume > 2) { // If out of range, default to 1
            _audioMixer_1 = 1;            
          } else { // Valid SETAM parameter in range
              _audioMixer_1 = volume; // _audioMixer_0 valid values 1 or 2
            }          
        break;
       default:
        break;
      }          
    }
  }

  // A read on any pin indicates if the player is still playing.
  int _read(VPIN vpin) override {
    if (_deviceState_0 == DEVSTATE_FAILED) return false;
    if (_deviceState_1 == DEVSTATE_FAILED) return false;
    uint8_t pin = vpin - _firstVpin;
      if (pin == 0) { // return _playing for uart 0
        return _playing_0;
      } else if (pin == 1) { // return _playing for uart 1
        return _playing_1;
        }
    }

  void _display() override {
    DIAG(F("I2CDFPlayer Configured on Vpins:%u-%u %S, UART: %d"), _firstVpin, _firstVpin+_nPins-1,
      (_deviceState_0==DEVSTATE_FAILED) ? F("OFFLINE") : F(""), _UART_0);
    DIAG(F("I2CDFPlayer Configured on Vpins:%u-%u %S, UART: %d"), _firstVpin, _firstVpin+_nPins-1,
      (_deviceState_1==DEVSTATE_FAILED) ? F("OFFLINE") : F(""), _UART_1);
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

  // TODO rewite sendPacket for both uarts -- Done
  void sendPacket(uint8_t command, uint8_t arg1 = 0, uint8_t arg2 = 0, uint8_t _uart = 0) {    
   if(_uart == _UART_0){
      FIFO_TX_LEVEL_0 = 0; // Reset FIFO_TX_LEVEL_0
    } else {
        FIFO_TX_LEVEL_1 = 0; // Reset FIFO_TX_LEVEL_1      
      }
    TX_fifo_lvl(_uart); // Now check for TX FIFO level    
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

      // Prepend the DFPlayer command with REG address and UART Channel in _outbuffer or _outbuffer
      _outbuffer[0] = 0x00; // empty _outbuffer[0] just in case
      if (_uart==_UART_0){
        _outbuffer[0] = REG_THR << 3 | _uart << 1; //TX FIFO and UART Channel      
        for ( int i = 1; i < sizeof(out)+1 ; i++){
          _outbuffer[i] = out[i-1];
        }
      } else if (_uart==_UART_1){
        _outbuffer[0] = REG_THR << 3 | _uart << 1; //TX FIFO and UART Channel      
        for ( int i = 1; i < sizeof(out)+1 ; i++){
          _outbuffer[i] = out[i-1];
        }
       }

      if (_uart==_UART_0){
        #ifdef DIAG_I2CDFplayer_data
          DIAG(F("SC16IS752: I2C: %s Sent packet function, UART: %d"), _I2CAddress.toString(), _uart);
          for (int i = 0; i < sizeof _outbuffer; i++){
            DIAG(F("SC16IS752: Data _outbuffer[0x%x]: 0x%x"), i, _outbuffer[i]);  
          }
        #endif
      } else if (_uart==_UART_1){
          #ifdef DIAG_I2CDFplayer_data
            DIAG(F("SC16IS752: I2C: %s Sent packet function, UART: %d"), _I2CAddress.toString(), _uart);
              for (int i = 0; i < sizeof _outbuffer; i++){
                DIAG(F("SC16IS752: Data _outbuffer[0x%x]: 0x%x"), i, _outbuffer[i]);  
            }
          #endif
      }      
    //TX_fifo_lvl(_uart); // wrong place, this will overwrite _oubuffer[0], moved to beginning of function
    if(_uart == _UART_0){
       if(FIFO_TX_LEVEL_0 > 10){ //FIFO uart 0 is empty, proceed
          I2CManager.write(_I2CAddress, _outbuffer, sizeof(_outbuffer), &_rb); // ************************* use this once buffer issue is solved *********************
          _commandSendTime_0 = micros();
          #ifdef DIAG_I2CDFplayer
            DIAG(F("SC16IS752: I2C: %s data transmit complete on UART: %d"), _I2CAddress.toString(), _uart);
          #endif
        } else {
          DIAG(F("I2CDFPlayer at: %s, TX FIFO not empty on UART: %d"), _I2CAddress.toString(), _uart);
          _deviceState_0 = DEVSTATE_FAILED; // This should not happen, _devstate_0 is for both UART 0
      }      
    } else { //Handle uart 1
        if(FIFO_TX_LEVEL_1 > 10){ //FIFO uart 1 is empty, proceed        
          I2CManager.write(_I2CAddress, _outbuffer, sizeof(_outbuffer), &_rb); // ************************* use this once buffer issue is solved *********************         
          _commandSendTime_1 = micros();
          #ifdef DIAG_I2CDFplayer
            DIAG(F("SC16IS752: I2C: %s data transmit complete on UART: %d"), _I2CAddress.toString(), _uart);
          #endif
        } else {
            DIAG(F("I2CDFPlayer at: %s, TX FIFO not empty on UART: %d"), _I2CAddress.toString(), _uart);
            _deviceState_1 = DEVSTATE_FAILED; // This should not happen, _devstate is for both UART 1
          }
      }
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
  // Initialise SC16IS752 only for this uart
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
  void Init_SC16IS752(uint8_t _uart){ // Return value is in _deviceState_0 or _deviceState_1
    #ifdef DIAG_I2CDFplayer
      DIAG(F("SC16IS752: Initialize I2C: %s , UART: %d"), _I2CAddress.toString(),  _uart);      
    #endif    
    UART_WriteRegister(REG_IOCONTROL, 0x08, _uart);                         // UART Software reset   
    UART_ReadRegister(REG_FCR, _uart);     
    UART_WriteRegister(REG_FCR, 0x07, _uart);                               // Reset FIFO, clear RX & TX FIFO    
    UART_WriteRegister(REG_MCR, 0x00, _uart);                               // Set MCR to all 0, includes Clock divisor
    TEMP_REG_VAL = 0x80 | WORD_LEN | STOP_BIT | PARITY_ENA | PARITY_TYPE;
    UART_WriteRegister(REG_LCR, TEMP_REG_VAL, _uart);                       // Divisor latch enabled
    uint16_t _divisor = (_sc16is752_xtal_freq/PRESCALER)/(BAUD_RATE * 16);  // Calculate _divisor for baudrate
    UART_WriteRegister(REG_DLL, (uint8_t)_divisor, _uart);                  // Write DLL
    UART_WriteRegister(REG_DLH, (uint8_t)(_divisor >> 8), _uart);           // Write DLH
    UART_ReadRegister(REG_LCR, _uart);                                      // Read LCR, _inbuffer_x[0] has value, then AND with 0x7F to set bit 7 to 0
    if (_uart == _UART_0){
      TEMP_REG_VAL = _inbuffer[0] & 0x7F;                                 // Disable Divisor latch enabled bit uart 0
    } else {
      if(_uart == _UART_1){
        TEMP_REG_VAL = _inbuffer[0] & 0x7F;                               // Disable Divisor latch enabled bit uart 1
      }
    }    
    UART_WriteRegister(REG_LCR, TEMP_REG_VAL, _uart);                       // Divisor latch disabled    
    UART_WriteRegister(REG_IOCONTROL, 0x00, _uart);                         // Set pins to GPIO mode    
    UART_WriteRegister(REG_IODIR, 0xFF, _uart);                             // Set all pins as output
    setGPIO(_uart);                                                         // Set the audio mixer channel, is uart independent

    status = _rb.status;
    if(_uart == _UART_0){ // only execute if _UART_0      
      if (status != I2C_STATUS_OK) {
        DIAG(F("SC16IS752: I2C: %s failed %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
        _deviceState_0 = DEVSTATE_FAILED;
      } else {
          #ifdef DIAG_IO
            DIAG(F("SC16IS752: I2C: %s, _rb.status: %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
          #endif
          _deviceState_0 = DEVSTATE_NORMAL; // If I2C state is OK, then proceed to initialize DFPlayer 
        }
    } else { // uart 1
      if (status != I2C_STATUS_OK) {
        DIAG(F("SC16IS752: I2C: %s failed %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
        _deviceState_1 = DEVSTATE_FAILED;
      } else {
          #ifdef DIAG_IO
            DIAG(F("SC16IS752: I2C: %s, _rb.status: %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
          #endif
          _deviceState_1 = DEVSTATE_NORMAL; // If I2C state is OK, then proceed to initialize DFPlayer 
        }
    }    
  }

  
  // Read the Receive FIFO Level register (RXLVL), return a single unsigned integer
  // of nr of characters in the RX FIFO, bit 6:0, 7 not used, set to zero
  // value from 0 (0x00) to 64 (0x40) Only display if RX FIFO has data
  // The RX fifo level is used to check if there are enough bytes to process a frame
  void RX_fifo_lvl(uint8_t _uart){
    UART_ReadRegister(REG_RXLV, _uart);
    if (_uart == _UART_0){
      FIFO_RX_LEVEL_0 = _inbuffer[0];
    } else { // uart 1    
        FIFO_RX_LEVEL_1 = _inbuffer[0];
      }
    #ifdef DIAG_I2CDFplayer
      if (_uart == _UART_0){
        if (FIFO_RX_LEVEL_0 > 0){
          //if (FIFO_RX_LEVEL_0 > 0 && FIFO_RX_LEVEL_0 < 10){ // uncomment if only RX lvl between 1-9 is required
        DIAG(F("SC16IS752: At I2C: %s, UART: 0d%d, FIFO_RX_LEVEL_0: 0d%d"), _I2CAddress.toString(), _uart, _inbuffer[0]);
        }
      } else { // uart 1
        if (FIFO_RX_LEVEL_1 > 0){
          //if (FIFO_RX_LEVEL_1 > 0 && FIFO_RX_LEVEL_1 < 10){ // uncomment if only RX lvl between 1-9 is required
        DIAG(F("SC16IS752: At I2C: %s, UART: 0d%d, FIFO_RX_LEVEL_1: 0d%d"), _I2CAddress.toString(), _uart, _inbuffer[0]);
        }
      }
    #endif   
  }

  // Read the Tranmit FIFO Level register (TXLVL), return a single unsigned integer
  // of nr characters free in the TX FIFO, bit 6:0, 7 not used, set to zero
  // value from 0 (0x00) to 64 (0x40)
  //
  void TX_fifo_lvl(uint8_t _uart){
    UART_ReadRegister(REG_TXLV, _uart);
    if(_uart == _UART_0){
      FIFO_TX_LEVEL_0 = _inbuffer[0];
    } else {
        FIFO_TX_LEVEL_1 = _inbuffer[0];
      }
    #ifdef DIAG_I2CDFplayer
    if(_uart == _UART_0){
    //  DIAG(F("SC16IS752: At I2C: %s, UART channel: 0d%d, FIFO_TX_LEVEL_0: 0d%d"), _I2CAddress.toString(), _uart, FIFO_TX_LEVEL_0);
    } else {
    //  DIAG(F("SC16IS752: At I2C: %s, UART channel: 0d%d, FIFO_TX_LEVEL_1: 0d%d"), _I2CAddress.toString(), _uart, FIFO_TX_LEVEL_1);
    }
    #endif 
  }

  // When a frame is transmitted from the DFPlayer to the serial port, and at the same time the CS is sending a 42 query
  // the following two frames from the DFPlayer are corrupt. This result in the receive buffer being out of sync and the 
  // CS will complain and generate a timeout.
  // The RX fifo has corrupt data and need to be flushed, this function does that
  // 
  void resetRX_fifo(uint8_t _uart){
    #ifdef DIAG_I2CDFplayer
      DIAG(F("SC16IS752: At I2C: %s, UART: %d, RX fifo reset"), _I2CAddress.toString(), _uart);
    #endif    
    //TEMP_REG_VAL = 0x03; // Reset RX fifo
    UART_WriteRegister(REG_FCR, 0x03, _uart);
  }

  // Set or reset GPIO pin 0 and 1 depending on the UART ch
  // This function may be modified in a future release to enable all 8 pins to be set or reset with EX-Rail
  // for various auxilary functions
  void setGPIO(uint8_t _uart){
    UART_ReadRegister(REG_IOSTATE, _uart); // Get the current GPIO pins state from the IOSTATE register, independant of uart
    // _inbuffer for uart 0, _inbuffer for uart 1
    if (_audioMixer_0 == 1){ // set to audio mixer 1      
      if (_uart == _UART_0){ 
        TEMP_REG_VAL = _inbuffer[0];
        TEMP_REG_VAL |= (0x01 << _uart); //Set GPIO pin 0 to high
        _setamCmd_0 = false;
      } else { // must be UART 1
         TEMP_REG_VAL = _inbuffer[0];
         TEMP_REG_VAL |= (0x01 << _uart); //Set GPIO pin 1 to high
         _setamCmd_1 = false;
        }        
    } else { // set to audio mixer 2
        if (_uart == _UART_0){
          TEMP_REG_VAL = _inbuffer[0]; 
          TEMP_REG_VAL &= ~(0x01 << _uart); //Set GPIO pin 0 to Low
          _setamCmd_0 = false;
        } else { // must be UART 1
           TEMP_REG_VAL = _inbuffer[0];
           TEMP_REG_VAL &= ~(0x01 << _uart); //Set GPIO pin 1 to Low
           _setamCmd_1 = false;
          }          
      }    
    UART_WriteRegister(REG_IOSTATE, TEMP_REG_VAL, _uart);
  }

 
  //void UART_WriteRegister(I2CAddress _I2CAddress, uint8_t _UART_CH, uint8_t UART_REG, uint8_t Val, uint8 _uart, I2CRB &_rb){
  void UART_WriteRegister(uint8_t UART_REG, uint8_t Val, uint8_t _uart){
    if (_uart == _UART_0){
      _outbuffer[0] = UART_REG << 3 | _uart << 1;
      _outbuffer[1] = Val;
      I2CManager.write(_I2CAddress, _outbuffer, 2);
    } else {
      if (_uart == _UART_1){
        _outbuffer[0] = UART_REG << 3 | _uart << 1;
        _outbuffer[1] = Val;
        I2CManager.write(_I2CAddress, _outbuffer, 2);
      }
    }       
    #ifdef DIAG_I2CDFplayer_reg
      if(_uart == _UART_0){
        DIAG(F("SC16IS752: Write register at I2C: %s, UART channel: %d, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _uart, UART_REG, _outbuffer[1]);
      } else {
          if(_uart == _UART_1){
          DIAG(F("SC16IS752: Write register at I2C: %s, UART channel: %d, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _uart, UART_REG, _outbuffer[1]);
        }  
      }
    #endif  
    }

 
  void UART_ReadRegister(uint8_t UART_REG, uint8_t _uart){    
     if (_uart == _UART_0){
      _outbuffer[0] = UART_REG << 3 | _uart << 1; // _outbuffer[0] has now UART_REG and UART
      I2CManager.read(_I2CAddress, _inbuffer, 1, _outbuffer, 1); 
     } else if (_uart == _UART_1){
        _outbuffer[0] = UART_REG << 3 | _uart << 1; // _outbuffer[0] has now UART_REG and UART
        I2CManager.read(_I2CAddress, _inbuffer, 1, _outbuffer, 1); 
      }
    // _inbuffer has the REG data
    #ifdef DIAG_I2CDFplayer_reg
      if (_uart == _UART_0){
        DIAG(F("SC16IS752: Read register at I2C: %s, UART: %d, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _uart, UART_REG, _inbuffer[0]);
      } else {
          DIAG(F("SC16IS752: Read register at I2C: %s, UART: %d, Register: 0x%x, Data: 0b%b"), _I2CAddress.toString(), _uart, UART_REG, _inbuffer[0]);
        }      
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
enum  : uint8_t{
    PLAY          = 0x0F,
    VOL           = 0x06,
    FOLDER        = 0x2B, // Not a DFPlayer command, used to set folder nr where audio file is
    REPEATPLAY    = 0x08,
    STOPPLAY      = 0x16,
    EQ            = 0x07, // Set equaliser, require parameter NORMAL, POP, ROCK, JAZZ, CLASSIC or BASS
    RESET         = 0x0C,
    DACON         = 0x1A,
    SETAM         = 0x2A, // Set audio mixer 1 or 2 for this DFPLayer   
    NORMAL        = 0x00, // Equalizer parameters
    POP           = 0x01,
    ROCK          = 0x02,
    JAZZ          = 0x03,
    CLASSIC       = 0x04,
    BASS          = 0x05,    
  };

};

#endif // IO_I2CDFPlayer_h
