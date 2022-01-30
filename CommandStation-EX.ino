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
#else
  #warning config.h not found. Using defaults from config.example.h
  #include "config.example.h"
#endif


/*
 *  © 2021 Neil McKechnie
 *  © 2020-2021 Chris Harlow, Harald Barth, David Cutting,
 *  Fred Decker, Gregor Baues, Anthony W - Dayton
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

  CONDITIONAL_LCD_START {
    // This block is still executed for DIAGS if LCD not in use
    LCD(0,F("DCC++ EX v%S"),F(VERSION));
    LCD(1,F("Lic GPLv3"));
  }

  // Responsibility 2: Start all the communications before the DCC engine
  // Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
  // Start Ethernet if it exists
#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL);
#endif // WIFI_ON

#if ETHERNET_ON
  EthernetInterface::setup();
#endif // ETHERNET_ON

  // Responsibility 3: Start the DCC engine.
  // Note: this provides DCC with two motor drivers, main and prog, which handle the motor shield(s)
  // Standard supported devices have pre-configured macros but custome hardware installations require
  //  detailed pin mappings and may also require modified subclasses of the MotorDriver to implement specialist logic.
  // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorShields.h
  DCC::begin(MOTOR_SHIELD_TYPE);

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

  LCD(3,F("Ready"));
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
#if WIFI_ON
  WifiInterface::loop();
#endif
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();  // ignored if no automation

  #if defined(LCN_SERIAL)
  LCN::loop();
  #endif

  LCDDisplay::loop();  // ignored if LCD not in use

  // Handle/update IO devices.
  IODevice::loop();

  Sensor::checkAll(); // Update and print changes

  // Report any decrease in memory (will automatically trigger on first call)
  static int ramLowWatermark = __INT_MAX__; // replaced on first loop

  int freeNow = minimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5db"), ramLowWatermark);
  }
}
