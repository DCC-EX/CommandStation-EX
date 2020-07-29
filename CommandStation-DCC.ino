/*
 *  main.cpp
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
#include <CommandStation.h>
#include <ArduinoTimers.h>

#include "Config.h"
#include "FreeMemory.h"

#if defined(ARDUINO_BOARD_UNO)
int ramLowWatermark = 16384;
#elif defined(ARDUINO_BOARD_MEGA_2560)
int ramLowWatermark = 64000; 
#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAMC)
int ramLowWatermark = 256000; 
#endif

const uint8_t kIRQmicros = 29;
const uint8_t kNumLocos = 50;

#if defined CONFIG_WSM_FIREBOX_MK1
DCCMain* mainTrack = DCCMain::Create_WSM_FireBox_MK1_Main(kNumLocos);
DCCService* progTrack = DCCService::Create_WSM_FireBox_MK1_Prog();
#elif defined CONFIG_WSM_FIREBOX_MK1S
DCCMain* mainTrack = DCCMain::Create_WSM_FireBox_MK1S_Main(kNumLocos);
DCCService* progTrack = DCCService::Create_WSM_FireBox_MK1S_Prog();
#elif defined CONFIG_ARDUINO_MOTOR_SHIELD
DCCMain* mainTrack = DCCMain::Create_Arduino_L298Shield_Main(kNumLocos);
DCCService* progTrack = DCCService::Create_Arduino_L298Shield_Prog();
#elif defined CONFIG_POLOLU_MOTOR_SHIELD
DCCMain* mainTrack = DCCMain::Create_Pololu_MC33926Shield_Main(kNumLocos);
DCCService* progTrack = DCCService::Create_Pololu_MC33926Shield_Prog();
#else
#error "Cannot compile - no board selected in Config.h"
#endif

void waveform_IrqHandler() {
  bool mainInterrupt = mainTrack->interrupt1();
  bool progInterrupt = progTrack->interrupt1();
  if(mainInterrupt) mainTrack->interrupt2();
  if(progInterrupt) progTrack->interrupt2();
}

#if defined(ARDUINO_ARCH_SAMD)
void SERCOM4_Handler()
{   
  mainTrack->railcom.getSerial()->IrqHandler();
}
#elif defined(ARDUINO_ARCH_SAMC)
void SERCOM0_Handler()
{   
  mainTrack->railcom.getSerial()->IrqHandler();
}
#endif

void setup() {
  mainTrack->setup();
  progTrack->setup();

  // TimerA is TCC0 on SAMD21, Timer1 on MEGA2560, and Timer1 on MEGA328
  // We will fire an interrupt every 29us to generate the signal on the track 
  TimerA.initialize();
  TimerA.setPeriod(kIRQmicros);
  TimerA.attachInterrupt(waveform_IrqHandler);
  TimerA.start();

  mainTrack->hdw.config_setTrackPowerCallback(DCCEXParser::trackPowerCallback);
  progTrack->hdw.config_setTrackPowerCallback(DCCEXParser::trackPowerCallback);

  // Register the serial interface
#if defined (ARDUINO_ARCH_SAMD)
  CommManager::registerInterface(new USBInterface(SerialUSB));
  while(!SerialUSB) {}  // Wait for USB to come online (remove once wifi is implemented)
  Wire.begin();       // Needed for EEPROM to work
#elif defined (ARDUINO_ARCH_SAMC)
  CommManager::registerInterface(new SerialInterface(Serial));
  Wire.begin();       // Needed for EEPROM to work
#elif defined(ARDUINO_ARCH_AVR)
  CommManager::registerInterface(new SerialInterface(Serial));
#endif

  EEStore::init();

  // Set up the string parser to accept commands from the interfaces
  DCCEXParser::init(mainTrack, progTrack);       

  CommManager::showInitInfo();           
}

void loop() {
  CommManager::update();
  mainTrack->loop();
  progTrack->loop();

  int freeNow=freeMemory();
  if (freeNow<ramLowWatermark) {
    ramLowWatermark=freeNow;
    CommManager::printf(F("\nFree RAM=%d\n"),ramLowWatermark);
  }
}
