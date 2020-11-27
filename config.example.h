/**********************************************************************

Config.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman
COPYRIGHT (c) 2020 Fred Decker

The configuration file for DCC++ EX Command Station

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
//   |
//   +-----------------------v
//
#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD

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
// If you do NOT set the WIFI_SSID, the WiFi chip will first try
// to connect to the previously configured network and if that fails
// fall back to Access Point mode. The SSID of the AP will be
// automatically set to DCCEX_*.
//
// Your SSID may not conain ``"'' (double quote, ASCII 0x22).
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
/////////////////////////////////////////////////////////////////////////////////////
//
// Wifi connect timeout in milliseconds. Default is 14000 (14 seconds). You only need
// to set this if you have an extremely slow Wifi router.
//
//#define WIFI_CONNECT_TIMEOUT 14000

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS *OR* COMMENT OUT TO USE DHCP
//
//#define IP_ADDRESS { 192, 168, 1, 200 }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MAC ADDRESS ARRAY FOR ETHERNET COMMUNICATIONS INTERFACE
//
// Uncomment to use with Ethernet Shields
//
// NOTE: This is not used with ESP8266 WiFi modules.
// 
// #define MAC_ADDRESS {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE LCD SCREEN USAGE BY THE BASE STATION
//
// Note: This feature requires an I2C enabled LCD screen using a PCF8574 based chipset.
//       or one using a Hitachi  HD44780.
//       OR an I2C Oled screen.
// To enable, uncomment one of the lines below

// define LCD_DRIVER for I2C LCD address 0x3f,16 cols, 2 rows
// #define LCD_DRIVER  0x3F,16,2

//OR define OLED_DRIVER width,height in pixels (address auto detected) 
// This will not work on a UNO due to memory constraints
// #define OLED_DRIVER 128,32

/////////////////////////////////////////////////////////////////////////////////////
//
// Enable warning as memory gets depleted
#define ENABLE_FREE_MEM_WARNING false
