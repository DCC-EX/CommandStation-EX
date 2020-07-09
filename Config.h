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

#ifndef COMMANDSTATION_DCC_CONFIG
#define COMMANDSTATION_DCC_CONFIG

// Choose the motor shield that you want to use.

//#define CONFIG_WSM_FIREBOX
#define CONFIG_ARDUINO_MOTOR_SHIELD
//#define CONFIG_POLOLU_MOTOR_SHIELD

//Define wifi settings here
#define CONFIG_ENABLE_WIFI
#define CONFIG_WIFI_SSID "Test"
#define CONFIG_WIFI_PASSWORD "password"
#define CONFIG_HOSTNAME "CommandStation"
#define CONFIG_MDNS_SERVERNAME "CommandStation"
#define CONFIG_SERVER_PORT 12090
#endif
