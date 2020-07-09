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

void waveform_IrqHandler()
{
  mainTrack->interruptHandler();
  progTrack->interruptHandler();
}

#if defined(ARDUINO_ARCH_SAMD)
void SERCOM4_Handler()
{
  mainTrack->railcom.getSerial()->IrqHandler();
}
Uart Serial1( &sercom2, 1, 0, SERCOM_RX_PAD_3, UART_TX_PAD_2) ;

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

#if defined(AVR_UNO)
  #define Serial1 ESPserial
#endif


#if defined(CONFIG_ENABLE_WIFI)
  Serial1.begin(115200);
  WifiInterface::setup(Serial1, F(CONFIG_WIFI_SSID), F(CONFIG_WIFI_PASSWORD), F(CONFIG_HOSTNAME), F(CONFIG_MDNS_SERVERNAME), CONFIG_SERVER_PORT);
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
