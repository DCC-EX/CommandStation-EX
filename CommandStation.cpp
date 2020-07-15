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

#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
#endif

#include "Config.h"

const uint8_t kIRQmicros = 29;
const uint8_t kNumLocos = 50;

#if defined CONFIG_WSM_FIREBOX
DCCMain *mainTrack = DCCMain::Create_WSM_FireBox_Main(kNumLocos);
DCCService *progTrack = DCCService::Create_WSM_FireBox_Prog();
#elif defined CONFIG_ARDUINO_MOTOR_SHIELD
DCCMain *mainTrack = DCCMain::Create_Arduino_L298Shield_Main(kNumLocos);
DCCService *progTrack = DCCService::Create_Arduino_L298Shield_Prog();
#elif defined CONFIG_POLOLU_MOTOR_SHIELD
DCCMain *mainTrack = DCCMain::Create_Pololu_MC33926Shield_Main(kNumLocos);
DCCService *progTrack = DCCService::Create_Pololu_MC33926Shield_Prog();
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

#endif

void setup()
{
  mainTrack->setup();
  progTrack->setup();

  // TimerA is TCC0 on SAMD21, Timer1 on MEGA2560, and Timer1 on MEGA328
  // We will fire an interrupt every 29us to generate the signal on the track
  TimerA.initialize();
  TimerA.setPeriod(kIRQmicros);
  TimerA.attachInterrupt(waveform_IrqHandler);
  TimerA.start();

#if defined(ARDUINO_ARCH_SAMD)
  CommManager::registerInterface(new USBInterface(SerialUSB));
  Wire.begin(); // Needed for EEPROM to work
#elif defined(ARDUINO_ARCH_AVR)
  CommManager::registerInterface(new SerialInterface(Serial));
#endif

#if defined(CONFIG_ENABLE_WIFI) && !defined(ARDUINO_AVR_UNO)
  CommManager::registerInterface(new WifiInterface(Serial1, F(CONFIG_WIFI_SSID), F(CONFIG_WIFI_PASSWORD), F(CONFIG_HOSTNAME), F(CONFIG_MDNS_SERVERNAME), CONFIG_SERVER_PORT));
#elif defined(CONFIG_ENABLE_WIFI) && defined(ARDUINO_AVR_UNO)
  SoftwareSerial wifiSerial(2, 3);
  CommManager::registerInterface(new WifiInterface(wifiSerial, F(CONFIG_WIFI_SSID), F(CONFIG_WIFI_PASSWORD), F(CONFIG_HOSTNAME), F(CONFIG_MDNS_SERVERNAME), CONFIG_SERVER_PORT));
#endif

  EEStore::init();

  // Set up the string parser to accept commands from the interfaces
  DCCEXParser::init(mainTrack, progTrack);

  CommManager::showInitInfo();
}

void loop()
{
  CommManager::update();
  mainTrack->loop();
  progTrack->loop();
}
