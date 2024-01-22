////////////////////////////////////////////////////////////////////////////////////
//  DCC-EX CommandStation-EX   Please see https://DCC-EX.com
//
// This file is the main sketch for the Command Station.
//
// CONFIGURATION:
// Configuration is normally performed by editing a file called config.h.
// This file is NOT shipped with the code so that if you pull a later version
// of the code, your configuration will not be overwritten.
//
// If you used the automatic installer program, config.h will have been created automatically.
//
// To obtain a starting copy of config.h please copy the file config.example.h which is
// shipped with the code and may be updated as new features are added.
//
// If config.h is not found, config.example.h will be used with all defaults.
////////////////////////////////////////////////////////////////////////////////////

#if __has_include ( "config.h")
  #include "config.h"
  #ifndef MOTOR_SHIELD_TYPE
  #error Your config.h must include a MOTOR_SHIELD_TYPE definition. If you see this warning in spite not having a config.h, you have a buggy preprocessor and must copy config.example.h to config.h
  #endif
#else
  #warning config.h not found. Using defaults from config.example.h
  #include "config.example.h"
#endif

/*
 *  © 2021 Neil McKechnie
 *  © 2020-2021 Chris Harlow, Harald Barth, David Cutting,
 *  Fred Decker, Gregor Baues, Anthony W - Dayton
 *  © 2023 Nathan Kellenicki
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
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

#include "DCCEX.h"
#include "Display_Implementation.h"

#ifdef CPU_TYPE_ERROR
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH THE ARCHITECTURES LISTED IN defines.h
#endif

#ifdef WIFI_WARNING
#warning You have defined that you want WiFi but your hardware has not enough memory to do that, so WiFi DISABLED
#endif
#ifdef ETHERNET_WARNING
#warning You have defined that you want Ethernet but your hardware has not enough memory to do that, so Ethernet DISABLED
#endif
#ifdef EXRAIL_WARNING
#warning You have myAutomation.h but your hardware has not enough memory to do that, so EX-RAIL DISABLED
#endif

void setup()
{
  // The main sketch has responsibilities during setup()

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  SerialManager::init();

  DIAG(F("License GPLv3 fsf.org (c) dcc-ex.com"));

// If user has defined a startup delay, delay here before starting IO
#if defined(STARTUP_DELAY)
  DIAG(F("Delaying startup for %dms"), STARTUP_DELAY);
  delay(STARTUP_DELAY);
#endif

// Initialise HAL layer before reading EEprom or setting up MotorDrivers 
  IODevice::begin();

  // As the setup of a motor shield may require a read of the current sense input from the ADC,
  // let's make sure to initialise the ADCee class!
  ADCee::begin();
  // Set up MotorDrivers early to initialize all pins
  TrackManager::Setup(MOTOR_SHIELD_TYPE);

  DISPLAY_START (
    // This block is still executed for DIAGS if display not in use
    LCD(0,F("DCC-EX v" VERSION));
    LCD(1,F("Lic GPLv3"));
  );

  // Responsibility 2: Start all the communications before the DCC engine
  // Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
  // Start Ethernet if it exists
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // WIFI_ON
#else
  // ESP32 needs wifi on always
  WifiESP::setup(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME, IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // ARDUINO_ARCH_ESP32

#if ETHERNET_ON
  EthernetInterface::setup();
#endif // ETHERNET_ON
  
  // Responsibility 3: Start the DCC engine.
  DCC::begin();

  // Start RMFT aka EX-RAIL (ignored if no automnation)
  RMFT::begin();


  // Invoke any DCC++EX commands in the form "SETUP("xxxx");"" found in optional file mySetup.h.
  //  This can be used to create turnouts, outputs, sensors etc. through the normal text commands.
  #if __has_include ( "mySetup.h")
    #define SETUP(cmd) DCCEXParser::parse(F(cmd))
    #include "mySetup.h"
    #undef SETUP
  #endif

  #if defined(LCN_SERIAL)
  LCN_SERIAL.begin(115200);
  LCN::init(LCN_SERIAL);
  #endif
  LCD(3, F("Ready"));
  CommandDistributor::broadcastPower();
}

void loop()
{
  // The main sketch has responsibilities during loop()

  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop();

  // Responsibility 2: handle any incoming commands on USB connection
  SerialManager::loop();

  // Responsibility 3: Optionally handle any incoming WiFi traffic
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::loop();
#endif //WIFI_ON
#else  //ARDUINO_ARCH_ESP32
#ifndef WIFI_TASK_ON_CORE0
  WifiESP::loop();
#endif
#endif //ARDUINO_ARCH_ESP32
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();  // ignored if no automation

  #if defined(LCN_SERIAL)
  LCN::loop();
  #endif

  // Display refresh
  DisplayInterface::loop();

  // Handle/update IO devices.
  IODevice::loop();

  Sensor::checkAll(); // Update and print changes

  // Report any decrease in memory (will automatically trigger on first call)
  static int ramLowWatermark = __INT_MAX__; // replaced on first loop

  int freeNow = DCCTimer::getMinimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5db"), ramLowWatermark);
  }
}
