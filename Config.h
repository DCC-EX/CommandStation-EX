/*
 *  Config.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMANDSTATION_DCC_CONFIG_H_
#define COMMANDSTATION_DCC_CONFIG_H_

#define VERSION "1.0.0"


// Choose the motor shield that you want to use.
#define CONFIG_ARDUINO_MOTOR_SHIELD
//#define CONFIG_POLOLU_MOTOR_SHIELD
//#define CONFIG_WSM_FIREBOX_MK1T     // Production version of FireBox
//#define CONFIG_WSM_FIREBOX_MK1A   // Early Beta version of FireBox

// Comment out this line to disable printing debug messages. Saves a lot of 
// flash when turned off. Needs to be turned off for WiFi on the UNO. 
#define DEBUG_MODE

// Define your WiFi credentials here, comment out WIFI_EN to disable WiFi.
//#define WIFI_EN
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
// WiFi AP uses the WIFI_PASSWORD and WIFI_HOSTNAME. Set hostname to something 
// unique, leave password empty for no security (not recommended)
#define WIFI_HOSTNAME "DCC-EX-12345"

// Macros translating board selection to board names
#if defined(CONFIG_ARDUINO_MOTOR_SHIELD)
#include "src/Boards/BoardArduinoMotorShield.h"
#define DCC_BOARD_NAME BoardArduinoMotorShield 
#define DCC_BOARD_CONFIG_NAME BoardConfigArduinoMotorShield
#define BOARD_NAME "Arduino Motor Shield"
#elif defined(CONFIG_POLOLU_MOTOR_SHIELD)
#include "src/Boards/BoardPololuMotorShield.h"
#define DCC_BOARD_NAME BoardPololuMotorShield 
#define DCC_BOARD_CONFIG_NAME BoardConfigPololuMotorShield 
#define BOARD_NAME "Pololu Motor Shield"
#elif defined(CONFIG_WSM_FIREBOX_MK1T)
#include "src/Boards/BoardWSMFireBoxMK1T.h"
#define DCC_BOARD_NAME BoardWSMFireBoxMK1T
#define DCC_BOARD_CONFIG_NAME BoardConfigWSMFireBoxMK1T
#define BOARD_NAME "WSM FireBox MK1T"
#elif defined(CONFIG_WSM_FIREBOX_MK1A)
#include "src/Boards/BoardWSMFireBoxMK1A.h"
#define DCC_BOARD_NAME BoardWSMFireBoxMK1A
#define DCC_BOARD_CONFIG_NAME BoardConfigWSMFireBoxMK1A
#define BOARD_NAME "WSM FireBox MK1A"
#else
#error "Config.h - you did not specify a valid board option"
#endif

#endif  // COMMANDSTATION_DCC_CONFIG_H_
