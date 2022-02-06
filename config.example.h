/*
 *  © 2021 Neil McKechnie
 *  © 2020-2021 Harald Barth
 *  © 2020-2021 Fred Decker
 *  © 2020-2021 Chris Harlow
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

/**********************************************************************

The configuration file for DCC-EX Command Station

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//  NOTE: Before connecting these boards and selecting one in this software
//        check the quick install guides!!! Some of these boards require a voltage
//        generating resitor on the current sense pin of the device. Failure to select
//        the correct resistor could damage the sense pin on your Arduino or destroy
//        the device.
//
// DEFINE MOTOR_SHIELD_TYPE BELOW ACCORDING TO THE FOLLOWING TABLE:
//
//  STANDARD_MOTOR_SHIELD : Arduino Motor shield Rev3 based on the L298 with 18V 2A per channel
//  POLOLU_MOTOR_SHIELD   : Pololu MC33926 Motor Driver (not recommended for prog track)
//  FUNDUMOTO_SHIELD      : Fundumoto Shield, no current sensing (not recommended, no short protection)
//  FIREBOX_MK1           : The Firebox MK1                    
//  FIREBOX_MK1S          : The Firebox MK1S
//  IBT_2_WITH_ARDUINO    : Arduino Motor Shield for PROG and IBT-2 for MAIN
//   |
//   +-----------------------v
//
//#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD

#define ESP_MOTOR_SHIELD F("ESP"),					\
 new MotorDriver(D3, D5, UNUSED_PIN, UNUSED_PIN, UNUSED_PIN, 2.99, 2000, UNUSED_PIN),\
 new MotorDriver(D2, D6, UNUSED_PIN, UNUSED_PIN, A0        , 2.99, 2000, UNUSED_PIN)

#define MOTOR_SHIELD_TYPE ESP_MOTOR_SHIELD

/////////////////////////////////////////////////////////////////////////////////////
//
// The IP port to talk to a WIFI or Ethernet shield.
//
#define IP_PORT 2560

/////////////////////////////////////////////////////////////////////////////////////
//
// NOTE: Only supported on Arduino Mega
// Set to false if you not even want it on the Arduino Mega
//
//#define ENABLE_WIFI true

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters (only in effect if WIFI is on)
//
// If DONT_TOUCH_WIFI_CONF is set, all WIFI config will be done with
// the <+> commands and this sketch will not change anything over
// AT commands and the other WIFI_* defines below do not have any effect.
//#define DONT_TOUCH_WIFI_CONF
//
// WIFI_SSID is the network name IF you want to use your existing home network.
// Do NOT change this if you want to use the WiFi in Access Point (AP) mode. 
//
// If you do NOT set the WIFI_SSID and do NOT set the WIFI_PASSWORD,
// then the WiFi chip will first try to connect to the previously
// configured network and if that fails fall back to Access Point mode.
// The SSID of the AP will be automatically set to DCCEX_*.
// If you DO set the WIFI_SSID then the WiFi chip will try to connect
// to that (home) network in station (client) mode. If a WIFI_PASSWORD
// is set (recommended), that password will be used for AP mode.
// The AP mode password must be at least 8 characters long.
//
// Your SSID may not contain ``"'' (double quote, ASCII 0x22).
#define WIFI_SSID "Your network name"
//
// WIFI_PASSWORD is the network password for your home network or if
// you want to change the password from default AP mode password
// to the AP password you want. 
// Your password may not conain ``"'' (double quote, ASCII 0x22).
#define WIFI_PASSWORD "Your network passwd"
//
// WIFI_HOSTNAME: You probably don't need to change this
#define WIFI_HOSTNAME "dccex"
//
// WIFI_CHANNEL: If the line "#define ENABLE_WIFI true" is uncommented, 
// WiFi will be enabled (Mega only). The default channel is set to "1" whether
// this line exists or not. If you need to use an alternate channel (we recommend
// using only 1,6, or 11) you may change it here.
#define WIFI_CHANNEL 1

/////////////////////////////////////////////////////////////////////////////////////
//
// ENABLE_ETHERNET: Set to true if you have an Arduino Ethernet card (wired). This
// is not for Wifi. You will then need the Arduino Ethernet library as well
//
//#define ENABLE_ETHERNET true


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//
//#define IP_ADDRESS { 192, 168, 1, 200 }


/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE LCD SCREEN USAGE BY THE BASE STATION
//
// Note: This feature requires an I2C enabled LCD screen using a Hitachi HD44780
//       controller and a commonly available PCF8574 based I2C 'backpack'.
// To enable, uncomment one of the #define lines below

// define LCD_DRIVER for I2C address 0x27, 16 cols, 2 rows
// #define LCD_DRIVER  0x27,16,2

//OR define OLED_DRIVER width,height in pixels (address auto detected)
// 128x32 or 128x64 I2C SSD1306-based devices are supported.
// Also 132x64 I2C SH1106 devices
// #define OLED_DRIVER 128,32

// Define scroll mode as 0, 1 or 2
#define SCROLLMODE 1

/////////////////////////////////////////////////////////////////////////////////////
// DISABLE EEPROM
//
// If you do not need the EEPROM at all, you can disable all the code that saves
// data in the EEPROM. You might want to do that if you are in a Arduino UNO
// and want to use the EX-RAIL automation. Otherwise you do not have enough RAM
// to do that. Of course, then none of the EEPROM related commands works.
//
// #define DISABLE_EEPROM

/////////////////////////////////////////////////////////////////////////////////////
// REDEFINE WHERE SHORT/LONG ADDR break is. According to NMRA the last short address
// is 127 and the first long address is 128. There are manufacturers which have
// another view. Lenz CS for example have considered addresses long from 100. If
// you want to change to that mode, do 
//#define HIGHEST_SHORT_ADDR 99
// If you want to run all your locos addressed long format, you could even do a 
//#define HIGHEST_SHORT_ADDR 0
// We do not support to use the same address, for example 100(long) and 100(short)
// at the same time, there must be a border.

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE TURNOUTS/ACCESSORIES FOLLOW NORM RCN-213
//
// According to norm RCN-213 a DCC packet with a 1 is closed/straight
// and one with a 0 is thrown/diverging.  In DCC++ Classic, and in previous
// versions of DCC++EX, a turnout throw command was implemented in the packet as 
// '1' and a close command as '0'. The #define below makes the states
// match with the norm.  But we don't want to cause havoc on existent layouts,
// so we define this only for new installations. If you don't want this,
// don't add it to your config.h.
//#define DCC_TURNOUTS_RCN_213

// By default, the driver which defines a DCC accessory decoder
// does send out the same state change on the DCC packet as it
// receives. This means a VPIN state=1 sends D=1 (close turnout
// or signal green) in the DCC packet. This can be reversed if
// necessary.
//#define HAL_ACCESSORY_COMMAND_REVERSE

// If you have issues with that the direction of the accessory commands is
// reversed (for example when converting from another CS to DCC-EX) then
// you can use this to revese the sense of all accessory commmands sent
// over DCC++. This #define likewise inverts the behaviour of the <a> command
// for triggering DCC Accessory Decoders, so that <a addr subaddr 0> generates a
// DCC packet with D=1 (close turnout) and <a addr subaddr 1> generates D=0 
// (throw turnout).
//#define DCC_ACCESSORY_RCN_213
//
// HANDLING MULTIPLE SERIAL THROTTLES
// The command station always operates with the default Serial port.
// Diagnostics are only emitted on the default serial port and not broadcast.
// Other serial throttles may be added to the Serial1, Serial2, Serial3 ports
// which may or may not exist on your CPU. (Mega has all 3)
// To monitor a throttle on one or more serial ports, uncomment the defines below.
// NOTE: do not define here the WiFi shield serial port or your wifi will not work.
//
//#define SERIAL1_COMMANDS
//#define SERIAL2_COMMANDS
//#define SERIAL3_COMMANDS

/////////////////////////////////////////////////////////////////////////////////////
