////////////////////////////////////////////////////////////////////////////////////
//  © 2020, Chris Harlow. All rights reserved.
//
//  This file is a demonstattion of setting up a DCC-EX
// Command station with optional support for direct connection of WiThrottle devices
// such as "Engine Driver". If you contriol your layout through JMRI
// then DON'T connect throttles to this wifi, connect them to JMRI.
//
//  THE WIFI FEATURE IS NOT SUPPORTED ON ARDUINO DEVICES WITH ONLY 2KB RAM.
////////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "DCCEX.h"


////////////////////////////////////////////////////////////////
//
// Enables an I2C 2x24 or 4x24 LCD Screen
#if ENABLE_LCD
bool lcdEnabled = false;
#if defined(LIB_TYPE_PCF8574)
LiquidCrystal_PCF8574 lcdDisplay(LCD_ADDRESS);
#elif defined(LIB_TYPE_I2C)
LiquidCrystal_I2C lcdDisplay = LiquidCrystal_I2C(LCD_ADDRESS, LCD_COLUMNS, LCD_LINES);
#endif
#endif

// Create a serial command parser for the USB connection, 
// This supports JMRI or manual diagnostics and commands
// to be issued from the USB serial console.
DCCEXParser serialParser;

void setup()
{
  
////////////////////////////////////////////
//
// More display stuff. Need to put this in a .h file and make
// it a class
#if ENABLE_LCD
  Wire.begin();
  // Check that we can find the LCD by its address before attempting to use it.
  Wire.beginTransmission(LCD_ADDRESS);
  if (Wire.endTransmission() == 0)
  {
    lcdEnabled = true;
    lcdDisplay.begin(LCD_COLUMNS, LCD_LINES);
    lcdDisplay.setBacklight(255);
    lcdDisplay.clear();
    lcdDisplay.setCursor(0, 0);
    lcdDisplay.print("DCC++ EX v");
    lcdDisplay.print(VERSION);
    lcdDisplay.setCursor(0, 1);
#if COMM_INTERFACE >= 1
    lcdDisplay.print("IP: PENDING");
#else
    lcdDisplay.print("SERIAL: READY");
#endif
#if LCD_LINES > 2
    lcdDisplay.setCursor(0, 3);
    lcdDisplay.print("TRACK POWER: OFF");
#endif
  }
#endif

  // The main sketch has responsibilities during setup()

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  Serial.begin(115200);

//  Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
//  NOTE: References to Serial1 are for the serial port used to connect
//        your wifi chip/shield.

#if WIFI_ON
  bool wifiUp = false;
  const __FlashStringHelper *wifiESSID = F(WIFI_SSID);
  const __FlashStringHelper *wifiPassword = F(WIFI_PASSWORD);
  const __FlashStringHelper *dccex = F(WIFI_HOSTNAME);
  const uint16_t port = IP_PORT;

  Serial1.begin(WIFI_SERIAL_LINK_SPEED);
  wifiUp = WifiInterface::setup(Serial1, wifiESSID, wifiPassword, dccex, port);
#if NUM_SERIAL > 1
  if (!wifiUp)
  {
    Serial2.begin(WIFI_SERIAL_LINK_SPEED);
    wifiUp = WifiInterface::setup(Serial2, wifiESSID, wifiPassword, dccex, port);
  }
#if NUM_SERIAL > 2
  if (!wifiUp)
  {
    Serial3.begin(WIFI_SERIAL_LINK_SPEED);
    wifiUp = WifiInterface::setup(Serial3, wifiESSID, wifiPassword, dccex, port);
  }
#endif // >2
#endif // >1
#endif // WIFI_ON

  // Responsibility 3: Start the DCC engine.
  // Note: this provides DCC with two motor drivers, main and prog, which handle the motor shield(s)
  // Standard supported devices have pre-configured macros but custome hardware installations require
  //  detailed pin mappings and may also require modified subclasses of the MotorDriver to implement specialist logic.

  // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorShields.h

  // Optionally a Timer number (1..4) may be passed to DCC::begin to override the default Timer1 used for the
  // waveform generation.  e.g.  DCC::begin(STANDARD_MOTOR_SHIELD,2); to use timer 2

  DCC::begin(MOTOR_SHIELD_TYPE);
}

void loop()
{
  // The main sketch has responsibilities during loop()

  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop();

  // Responsibility 2: handle any incoming commands on USB connection
  serialParser.loop(Serial);

// Responsibility 3: Optionally handle any incoming WiFi traffic
#if WIFI_ON
  WifiInterface::loop();
#endif
 
// Optionally report any decrease in memory (will automatically trigger on first call)
#if ENABLE_FREE_MEM_WARNING
  static int ramLowWatermark = 32767; // replaced on first loop 

  int freeNow = freeMemory();
  if (freeNow < ramLowWatermark)
  {
    ramLowWatermark = freeNow;
    DIAG(F("\nFree RAM=%d\n"), ramLowWatermark);
  }
#endif
}
