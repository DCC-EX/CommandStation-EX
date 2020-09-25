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
// DEFINE MOTOR_SHIELD_TYPE ACCORDING TO THE FOLLOWING TABLE:
//
//  STANDARD_MOTOR_SHIELD = ARDUINO MOTOR SHIELD            (MAX 18V/2A  PER CHANNEL)  Arduino Motor shield Rev3 based on the L298
//  POLOLU_MOTOR_SHIELD = POLOLU MC33926 MOTOR SHIELD     (MAX 28V/2.5 PER CHANNEL)  Pololu MC33926 Motor Driver (shield or carrier)
//  FUNDUMOTO_SHIELD = FunduMoto Motor Shield                     
//  FIREBOX_MK1 = Firebox MK1                    
//  FIREBOX_MK1S = Firebox MK1S    


#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE PROGRAM TRACK CURRENT LIMIT IN MILLIAMPS

#define TRIP_CURRENT_PROG 250

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
#define WIFI_SSID "Your network name"
#define WIFI_PASSWORD "Your network passwd"
#define WIFI_HOSTNAME "dccex"

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

// #define MAC_ADDRESS {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF }

/////////////////////////////////////////////////////////////////////////////////////
//
// Allows using a pin as a trigger for a scope or analyzer so we can capture only
// the important parts of the data stream
//
// USE_TRIGGERPIN: Enable code that switches the trigger pin on and off at end
//                 of the preamble. This takes some clock cycles in the
//                 interrupt routine for the main track.
// USE_TRIGGERPIN_PER_BIT: As above but for every bit. This only makes sense
//                 if USE_TRIGGERPIN is set.
//
// The value of the TRIGGERPIN is defined in DCCppEX.h because it might
// be board specific
//
//#define USE_TRIGGERPIN
//#define USE_TRIGGERPIN_PER_BIT

/////////////////////////////////////////////////////////////////////////////////////
//
// Define only of you need the store to EEPROM feature. This takes RAM and
// you may need to use less MAX_MAIN_REGISTERS to compensate (at least on the UNO)

#define EESTORE

/////////////////////////////////////////////////////////////////////////////////////
//
// This shows the status and version at startup. This takes RAM. You can comment
// this line if you need to increase MAX_MAIN_REGISTERS(at least on the UNO)

#define SHOWCONFIG

/////////////////////////////////////////////////////////////////////////////////////
//
// This is different from the above config display which only shows one line at startup
// This defines a pin that when jumpered to ground before powering up the Arduinio, 
// will display more detailed settings for diagnostics. You must remove the jumper and
// restart the Arduino to return to normal operation

#define SHOW_CONFIG_DETAIL_PIN A2

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE LCD SCREEN USAGE BY THE BASE STATION
//
// Note: This feature requires an I2C enabled LCD screen using a PCF8574 based chipset.
//       or one using a Hitachi  HD44780.
//
// To enable, uncomment the line below and make sure only the correct LIB_TYPE line
// is uncommented below to select the library used for your LCD backpack

//#define ENABLE_LCD

#ifdef ENABLE_LCD
    #define LIB_TYPE_PCF8574
	//#define LIB_TYPE_I2C
	// This defines the I2C address for the LCD device
	#define LCD_ADDRESS 0x27 //common defaults are 0x27 and 0x3F

	// This defines the number of columns the LCD device has
	#define LCD_COLUMNS 16

	// This defines the number of lines the LCD device has
	#define LCD_LINES 2
#endif


/////////////////////////////////////////////////////////////////////////////////////
//
// Enable custom command filtering
#define ENABLE_CUSTOM_FILTER false

/////////////////////////////////////////////////////////////////////////////////////
//
// Enable custom command filtering
#define ENABLE_CUSTOM_CALLBACK false

/////////////////////////////////////////////////////////////////////////////////////
//
// Enable custom command filtering
#define ENABLE_FREE_MEM_WARNING false
