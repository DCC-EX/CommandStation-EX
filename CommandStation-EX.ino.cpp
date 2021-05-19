# 1 "/var/folders/3z/5trn2qms2h75fpmtlsywh1c80000gn/T/tmplx9j4cov"
#include <Arduino.h>
# 1 "/Users/gregor/Projects/CommandStation-EX/CommandStation-EX.ino"
# 19 "/Users/gregor/Projects/CommandStation-EX/CommandStation-EX.ino"
#if __has_include("config.h")
#include "config.h"
#else
#warning config.h not found. Using defaults from config.example.h
#include "config.example.h"
#endif
# 45 "/Users/gregor/Projects/CommandStation-EX/CommandStation-EX.ino"
#include "DCCEX.h"




DCCEXParser serialParser;
void setup();
void loop();
#line 52 "/Users/gregor/Projects/CommandStation-EX/CommandStation-EX.ino"
void setup()
{




  Serial.begin(115200);

  CONDITIONAL_LCD_START
  {

    LCD(0, F("DCC++ EX v%S"), F(VERSION));
    LCD(1, F("Starting"));
  }



#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL);
#endif

#if ETHERNET_ON
  EthernetInterface::setup();
#endif

#if MQTT_ON
  MQTTInterface::setup();
#endif
# 88 "/Users/gregor/Projects/CommandStation-EX/CommandStation-EX.ino"
  DCC::begin(MOTOR_SHIELD_TYPE);

#if defined(RMFT_ACTIVE)
  RMFT::begin();
#endif

#if __has_include("mySetup.h")
#define SETUP(cmd) serialParser.parse(F(cmd))
#include "mySetup.h"
#undef SETUP
#endif

#if defined(LCN_SERIAL)
  LCN_SERIAL.begin(115200);
  LCN::init(LCN_SERIAL);
#endif



  LCD(1, F("Ready"));
}

void loop()
{




  DCC::loop();


  serialParser.loop(Serial);


#if WIFI_ON
  WifiInterface::loop();
#endif

#if ETHERNET_ON
  EthernetInterface::loop();
#endif

#if MQTT_ON
  MQTTInterface::loop();
#endif

#if defined(RMFT_ACTIVE)
  RMFT::loop();
#endif

#if defined(LCN_SERIAL)
  LCN::loop();
#endif

  LCDDisplay::loop();


  static int ramLowWatermark = __INT_MAX__;

  int freeNow = minimumFreeMemory();
  if (freeNow < ramLowWatermark)
  {
    ramLowWatermark = freeNow;
    LCD(2, F("Free RAM=%5db"), ramLowWatermark);
  }
}