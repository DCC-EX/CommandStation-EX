/*
 *  © 2022 Paul M. Antoine
 *  © 2021 Neil McKechnie
 *  © 2020-2023 Harald Barth
 *  © 2020-2021 Fred Decker
 *  © 2020-2021 Chris Harlow
 *  © 2023 Nathan Kellenicki
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
// If you want to add your own motor driver definition(s), add them here
//   For example MY_SHIELD with display name "MINE":
//   (remove comment start and end marker if you want to edit and use that)
/* 
#define MY_SHIELD F("MINE"), \
 new MotorDriver( 3, 12, UNUSED_PIN, 9, A0, 5.08, 3000, A4), \
 new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 5.08, 1500, A5)
*/

/////////////////////////////////////////////////////////////////////////////////////
//  NOTE: Before connecting these boards and selecting one in this software
//        check the quick install guides!!! Some of these boards require a voltage
//        generating resistor on the current sense pin of the device. Failure to select
//        the correct resistor could damage the sense pin on your Arduino or destroy
//        the device.
//
// DEFINE MOTOR_SHIELD_TYPE BELOW. THESE ARE EXAMPLES. FULL LIST IN MotorDrivers.h
//
//  STANDARD_MOTOR_SHIELD : Arduino Motor shield Rev3 based on the L298 with 18V 2A per channel
//  POLOLU_MOTOR_SHIELD   : Pololu MC33926 Motor Driver (not recommended for prog track)
//  FUNDUMOTO_SHIELD      : Fundumoto Shield, no current sensing (not recommended, no short protection)
//  FIREBOX_MK1           : The Firebox MK1                    
//  FIREBOX_MK1S          : The Firebox MK1S
//  IBT_2_WITH_ARDUINO    : Arduino Motor Shield for PROG and IBT-2 for MAIN
//  EX8874_SHIELD         : DCC-EX TI DRV8874 based motor shield
//   |
//   +-----------------------v
//
#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
//
/////////////////////////////////////////////////////////////////////////////////////
//
// If you want to restrict the maximum current LOWER than what your
// motor shield can provide, you can do that here. For example if you
// have a motor shield that can provide 5A and your power supply can
// only provide 2.5A then you should restict the maximum current to
// 2.25A (90% of 2.5A) so that DCC-EX does shut off the track before
// your PS does shut DCC-EX. MAX_CURRENT is in mA so for this example
// it would be 2250, adjust the number according to your PS. If your
// PS has a higher rating than your motor shield you do not need this.
// You can use this as well if you are cautious and your trains do not
// need full current.
// #define MAX_CURRENT 2250
//
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
#define ENABLE_WIFI true

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
// Your password may not contain ``"'' (double quote, ASCII 0x22).
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
//
// WIFI_FORCE_AP: If you'd like to specify your own WIFI_SSID in AP mode, set this
// true. Otherwise it is assumed that you'd like to connect to an existing network
// with that SSID.
#define WIFI_FORCE_AP false

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

//OR define OLED_DRIVER width,height[,address] in pixels (address auto detected if not supplied)
// 128x32 or 128x64 I2C SSD1306-based devices are supported.
// Use 132,64 for a SH1106-based I2C device with a 128x64 display.
// #define OLED_DRIVER 0x3c,128,32

// Define scroll mode as 0, 1 or 2
//  *  #define SCROLLMODE 0 is scroll continuous (fill screen if poss),
//  *  #define SCROLLMODE 1 is by page (alternate between pages),
//  *  #define SCROLLMODE 2 is by row (move up 1 row at a time).
#define SCROLLMODE 1

// In order to avoid wasting memory the current scroll buffer is limited
// to 8 lines.  Some users wishing to display additional information
// such as TrackManager power states have requested additional rows aware
// of the warning that this will take extra RAM.  if you wish to include additional rows
// uncomment the following #define and set the number of lines you need.
//#define MAX_CHARACTER_ROWS 12


/////////////////////////////////////////////////////////////////////////////////////
// DISABLE EEPROM
//
// If you do not need the EEPROM at all, you can disable all the code that saves
// data in the EEPROM. You might want to do that if you are in a Arduino UNO
// and want to use the EXRAIL automation. Otherwise you do not have enough RAM
// to do that. Of course, then none of the EEPROM related commands work.
//
// EEPROM does not work on ESP32. So on ESP32, EEPROM will always be disabled,
// at least until it works.
//
// #define DISABLE_EEPROM

/////////////////////////////////////////////////////////////////////////////////////
// DISABLE PROG
//
// If you do not need programming capability, you can disable all programming related
// commands. You might want to do that if you are using an Arduino UNO and still want
// to use EXRAIL automation, as the Uno is lacking in RAM and Flash to run both.
// 
// Note this disables all programming functionality, including EXRAIL.
//
// #define DISABLE_PROG

/////////////////////////////////////////////////////////////////////////////////////
// DISABLE / ENABLE VDPY
//
// The Virtual display "VDPY" feature is by default enabled everywhere
// but on Uno and Nano. If you think you can fit it (for example
// having disabled some of the features above) you can enable it with
// ENABLE_VDPY. You can even disable it on all other CPUs with
// DISABLE_VDPY
//
// #define DISABLE_VDPY
// #define ENABLE_VDPY

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
// Some newer 32bit microcontrollers boot very quickly, so powering on I2C and other
// peripheral devices at the same time may result in the CommandStation booting too
// quickly to detect them.
// To work around this, uncomment the STARTUP_DELAY line below and set a value in
// milliseconds that works for your environment, default is 3000 (3 seconds).
// #define STARTUP_DELAY 3000

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
// you can use this to reverse the sense of all accessory commmands sent
// over DCC++. This #define likewise inverts the behaviour of the <a> command
// for triggering DCC Accessory Decoders, so that <a addr subaddr 0> generates a
// DCC packet with D=1 (close turnout) and <a addr subaddr 1> generates D=0 
// (throw turnout).
//#define DCC_ACCESSORY_RCN_213
//
// HANDLING MULTIPLE SERIAL THROTTLES
// The command station always operates with the default Serial port.
// Diagnostics are only emitted on the default serial port and not broadcast.
// Other serial throttles may be added to the Serial1, Serial2, Serial3, Serial4,
// Serial5, and Serial6 ports which may or may not exist on your CPU. (Mega has 3,
// SAMD/SAMC and STM32 have up to 6.)
// To monitor a throttle on one or more serial ports, uncomment the defines below.
// NOTE: do not define here the WiFi shield serial port or your wifi will not work.
//
//#define SERIAL1_COMMANDS
//#define SERIAL2_COMMANDS
//#define SERIAL3_COMMANDS
//#define SERIAL4_COMMANDS
//#define SERIAL5_COMMANDS
//#define SERIAL6_COMMANDS
//
// BLUETOOTH SERIAL ON ESP32
// On ESP32 you have the possibility to use the builtin BT serial to connect to
// the CS.
//
// The CS shows up as a pairable BT Clasic device. Name is "DCCEX-hexnumber".
// BT is as an additional serial port, debug messages are still sent over USB,
// not BT serial.
//
// If you enable this there are some implications:
// 1. WiFi will sleep more (as WiFi and BT share the radio. So WiFi performance
//    may suffer
// 2. The app will be bigger that 1.2MB, so the default partition scheme will not
//    work any more. You need to choose a partition scheme with 2MB (or bigger).
//    For example "NO OTA (2MB APP, 2MB SPIFFS)" in the Arduino IDE.
// 3. There is no securuity (PIN) implemented. Everyone in radio range can pair
//    with your CS.
//
//#define SERIAL_BT_COMMANDS

// BOOSTER PIN INPUT ON ESP32
// On ESP32 you have the possibility to define a pin as booster input
// Arduio pin D2 is GPIO 26 on ESPDuino32
//
//#define BOOSTER_INPUT 26

// SABERTOOTH
//
// This is a very special option and only useful if you happen to have a
// sabertooth motor controller from dimension engineering configured to
// take commands from and ESP32 via serial at 9600 baud from GPIO17 (TX)
// and GPIO16 (RX, currently unused).
// The number defined is the DCC address for which speed controls are sent
// to the sabertooth controller _as_well_. Default: Undefined.
//
//#define SABERTOOTH 1

/////////////////////////////////////////////////////////////////////////////////////
