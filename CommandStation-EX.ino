/*
 *  CommandStation-EX.ino
 * 
 *  This file is part of CommandStation-DCC.
 *
 *  CommandStation-DCC is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-DCC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-DCC.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include "src/DCC/DCCMain.h"
#include "src/DCC/DCCService.h"
#include "src/CommInterface/CommManager.h"
#include "src/CommInterface/DCCEXParser.h"
#include "src/Accessories/EEStore.h"
#include "src/ArduinoTimers/ArduinoTimers.h"

#include "Config.h"
#include "src/Utils/FreeMemory.h"

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
int ramLowWatermark = 32767;
#else
int ramLowWatermark = 256000;
#endif

const uint8_t kIRQmicros = 29;
const uint8_t kNumLocos = 50;

Railcom* mainRailcom;

DCC_BOARD_NAME* mainBoard;
DCC_BOARD_NAME* progBoard;

DCCMain* mainTrack;
DCCService* progTrack;

void waveform_IrqHandler() {
  bool mainInterrupt = mainTrack->interrupt1();
  bool progInterrupt = progTrack->interrupt1();
  if(mainInterrupt) mainTrack->interrupt2();
  if(progInterrupt) progTrack->interrupt2();
}

void setup() {
  DCC_BOARD_CONFIG_NAME mainConfig;
  DCC_BOARD_NAME::getDefaultConfigA(mainConfig);
  mainConfig.track_power_callback = DCCEXParser::trackPowerCallback;
  ////////////////////////////////////////////////////////////////////////////////////////
  // Add modifications to pinouts, currents, etc here using mainConfig.SETTING = VALUE; //
  ////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////
  mainBoard = new DCC_BOARD_NAME(mainConfig);

  DCC_BOARD_CONFIG_NAME progConfig;
  DCC_BOARD_NAME::getDefaultConfigB(progConfig);
  progConfig.track_power_callback = DCCEXParser::trackPowerCallback;
  ////////////////////////////////////////////////////////////////////////////////////////
  // Add modifications to pinouts, currents, etc here using progConfig.SETTING = VALUE; //
  ////////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////////
  progBoard = new DCC_BOARD_NAME(progConfig);

  RailComConfig rcomConfig;
  Railcom::getDefaultConfig(rcomConfig);  // Default is off
  mainRailcom = new Railcom(rcomConfig);

  mainBoard->setup();
  mainTrack = new DCCMain(kNumLocos, mainBoard, mainRailcom);
  mainTrack->setup();
  
  progBoard->setup();
  progTrack = new DCCService(progBoard);
  progTrack->setup(); // Currently doesn't do anything, but may be extended later
  progTrack->board->progMode(ON);   // Limits current to 250mA. Current limit can be changed in config above.

  // TimerA is TCC0 on SAMD21, Timer1 on MEGA2560, and Timer1 on MEGA328
  // We will fire an interrupt every 29us to generate the signal on the track 
  TimerA.initialize();
  TimerA.setPeriod(kIRQmicros);
  TimerA.attachInterrupt(waveform_IrqHandler);
  TimerA.start();

  // Register the serial interface
#if defined (ARDUINO_ARCH_SAMD)
  CommManager::registerInterface(new USBInterface(SerialUSB));
  while(!SerialUSB) {}
  Wire.begin();       // Needed for EEPROM to work
  EEStore::init(&SerialUSB);
#elif defined (ARDUINO_ARCH_SAMC)
  CommManager::registerInterface(new SerialInterface(Serial));
  // Wire.begin();       // Needed for EEPROM to work
  // EEStore::init(&Serial);
#elif defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
  CommManager::registerInterface(new SerialInterface(Serial));
  EEStore::init(&Serial);
#endif

  // Set up the string parser to accept commands from the interfaces
  DCCEXParser::init(mainTrack, progTrack);       

  CommManager::showInitInfo();           
}

void loop() {
  CommManager::update();
  mainTrack->loop();
  progTrack->loop();

#if defined(FREE_MEM_PRINT)
  int freeNow=freeMemory();
  if (freeNow<ramLowWatermark) {
    ramLowWatermark=freeNow;
    CommManager::broadcast(F("\n\rFree RAM=%d\n\r"),ramLowWatermark);
  }
#endif
}