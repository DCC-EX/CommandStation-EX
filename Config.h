/*
 *  Config.h
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

#ifndef COMMANDSTATION_DCC_CONFIG_H_
#define COMMANDSTATION_DCC_CONFIG_H_

#define VERSION "1.0.0"
#define BOARD_NAME "DCC++ CommandStation"

// Choose the motor shield that you want to use.
//#define CONFIG_ARDUINO_MOTOR_SHIELD
//#define CONFIG_POLOLU_MOTOR_SHIELD
//#define CONFIG_WSM_FIREBOX_MK1T     // Production version of FireBox
#define CONFIG_WSM_FIREBOX_MK1A   // Early Beta version of FireBox

// Comment out this line to disable printing debug messages
#define DEBUG_MODE

// Define your WiFi credentials here, comment out WIFI_EN to disable WiFi.
#define WIFI_EN
#define WIFI_SSID "1"
#define WIFI_PASSWORD "2"

// Define the pin mappings for the wifi software serial here (UNO only)
#define SS_RX_PIN 16
#define SS_TX_PIN 17

// Macros translating board selection to board names
#if defined(CONFIG_ARDUINO_MOTOR_SHIELD)
#include "src/Boards/BoardArduinoMotorShield.h"
#define DCC_BOARD_NAME BoardArduinoMotorShield 
#define DCC_BOARD_CONFIG_NAME BoardConfigArduinoMotorShield
#elif defined(CONFIG_POLOLU_MOTOR_SHIELD)
#include "src/Boards/BoardPololuMotorShield.h"
#define DCC_BOARD_NAME BoardPololuMotorShield 
#define DCC_BOARD_CONFIG_NAME BoardConfigPololuMotorShield 
#elif defined(CONFIG_WSM_FIREBOX_MK1T)
#include "src/Boards/BoardWSMFireBoxMK1T.h"
#define DCC_BOARD_NAME BoardWSMFireBoxMK1T
#define DCC_BOARD_CONFIG_NAME BoardConfigWSMFireBoxMK1T
#elif defined(CONFIG_WSM_FIREBOX_MK1A)
#include "src/Boards/BoardWSMFireBoxMK1A.h"
#define DCC_BOARD_NAME BoardWSMFireBoxMK1A
#define DCC_BOARD_CONFIG_NAME BoardConfigWSMFireBoxMK1A
#else
#error "Config.h - you did not specify a valid board option"
#endif

#endif  // COMMANDSTATION_DCC_CONFIG_H_
