# 1 "C:\\Users\\Fred\\AppData\\Local\\Temp\\tmp3tmcz0nh"
#include <Arduino.h>
# 1 "C:/Users/Fred/Documents/PlatformIO/Projects/CommandStation-EX/CommandStation-EX.ino"
# 19 "C:/Users/Fred/Documents/PlatformIO/Projects/CommandStation-EX/CommandStation-EX.ino"
#if __has_include ( "config.h")
  #include "config.h"
#else
  #warning config.h not found. Using defaults from config.example.h
  #include "config.example.h"
#endif
# 49 "C:/Users/Fred/Documents/PlatformIO/Projects/CommandStation-EX/CommandStation-EX.ino"
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
void setup();
void loop();
#line 60 "C:/Users/Fred/Documents/PlatformIO/Projects/CommandStation-EX/CommandStation-EX.ino"
void setup()
{




  SerialManager::init();

  DIAG(F("License GPLv3 fsf.org (c) dcc-ex.com"));

  CONDITIONAL_LCD_START {

    LCD(0,F("DCC++ EX v%S"),F(VERSION));
    LCD(1,F("Lic GPLv3"));
  }




#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL);
#endif

#if ETHERNET_ON
  EthernetInterface::setup();
#endif






  DCC::begin(MOTOR_SHIELD_TYPE);


  RMFT::begin();




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




  DCC::loop();


  SerialManager::loop();


#if WIFI_ON
  WifiInterface::loop();
#endif
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();

  #if defined(LCN_SERIAL)
  LCN::loop();
  #endif

  LCDDisplay::loop();


  IODevice::loop();

  Sensor::checkAll();


  static int ramLowWatermark = __INT_MAX__;

  int freeNow = minimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5db"), ramLowWatermark);
  }
}