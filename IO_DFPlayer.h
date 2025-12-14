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
 */

#ifndef IO_DFPlayer_h
#define IO_DFPlayer_h
#include "IO_DFPlayerSerial.h"
#include "IO_DFPlayerI2C.h"

class DFPlayer : public IODevice {

  // This class contains the HAL create calls for DFPlayer devices.
  // Combinations of parameters determine the actual class to be loaded.

  // NON-ESP32 Serial:
  //   HAL(DFPlayer, firstVpin, Serialx)          
  //   HAL(DFPlayer, firstVpin, nPins, Serialx)  - [Deprecated] 
  
  // ESP32 Serial:
  //   HAL(DFPlayer, firstVpin, Serialx, rxPin,txPin)  - Requires pins to begin serial
  //   HAL(DFPlayer, firstVpin, nPins, Serialx, rxPin,txPin)  - [Deprecated]

  // I2C connected via SC16IS752  (TODO second uart) 
  //   HAL(DFPlayer, firstVpin, I2CAddress)
  //   HAL(DFPlayer, firstVpin, npins, I2CAddress)  - Reserved for futute use

public:

#ifndef ESP32
  // NON-ESP32 Serial:
  //   HAL(DFPlayer, firstVpin, Serialx)  
  static void create(VPIN firstVpin, HardwareSerial &serial) {
    create(firstVpin, 1, serial);
  }

  //   HAL(DFPlayer, firstVpin, nPins, Serialx)  - [Deprecated] 
   static void create(VPIN firstVpin, int nPins, HardwareSerial &serial) {
    if (checkNoOverlap(firstVpin,nPins)) {
      serial.begin(9600, SERIAL_8N1); // 9600baud, no parity, 1 stop bit
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }
#endif

#ifdef ESP32
  // ESP32 Serial:
  //   HAL(DFPlayer, firstVpin, Serialx, rxPin,txPin)  - Requires pins to begin serial
  static void create(VPIN firstVpin,  HardwareSerial &serial, int8_t rxPin, int8_t txPin) {
    create(firstVpin, 1, serial, rxPin, txPin);
  }

  //   HAL(DFPlayer, firstVpin, nPins, Serialx, rxPin,txPin)  - [Deprecated]
  static void create(VPIN firstVpin, int nPins, HardwareSerial &serial,
    int8_t rxPin, int8_t txPin) {
    if (checkNoOverlap(firstVpin,nPins)) {
      serial.begin(9600, SERIAL_8N1,rxPin,txPin); // 9600baud, no parity, 1 stop bit
      new DFPlayerSerial(firstVpin, nPins, serial);
    }
  }


#endif

  // I2C connected via SC16IS752  (TODO second uart) 
  //   HAL(DFPlayer, firstVpin, I2CAddress)
   static void create(VPIN firstVpin, I2CAddress i2cAddress, uint8_t xtal) {
    create(firstVpin, 1, i2cAddress, xtal); // Default to 1 pins
    }

   //   HAL(DFPlayer, firstVpin, npins, I2CAddress)
   static void create(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint8_t xtal) {
    if (checkNoOverlap(firstVpin, nPins, i2cAddress)) 
      new DFPlayerI2C(firstVpin, nPins, i2cAddress, xtal); 
    }

  };
#endif // IO_DFPlayer_h
