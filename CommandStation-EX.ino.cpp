# 1 "C:\\Users\\chris\\AppData\\Local\\Temp\\tmpmrl7nc6z"
#include <Arduino.h>
# 1 "C:/GitHub/CommandStation-EX/CommandStation-EX.ino"
# 16 "C:/GitHub/CommandStation-EX/CommandStation-EX.ino"
#if __has_include ( "config.h")
  #include "config.h"
#else
  #warning config.h not found.
#endif
#ifndef MOTOR_SHIELD_TYPE
  #warning MOTOR_SHIELD_TYPE not found. Building with no motor shield
  #define MOTOR_SHIELD_TYPE NO_SHIELD
#endif
# 50 "C:/GitHub/CommandStation-EX/CommandStation-EX.ino"
#include "DCCEX.h"
#include "Display_Implementation.h"
#ifdef ARDUINO_ARCH_ESP32
#include "Sniffer.h"
#include "DCCDecoder.h"
Sniffer *dccSniffer = NULL;
bool DCCDecoder::active = false;
#endif

#define QWRAP_(x) QWRAP__(x)
#define QWRAP__(x) #x
static_assert(MAX_LOCOS >1 && MAX_LOCOS<256, "#define MAX_LOCOS " QWRAP_(MAX_LOCOS) " must be >1 and <256");

#ifdef CPU_TYPE_ERROR
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH THE ARCHITECTURES LISTED IN defines.h
#endif

#ifdef WIFI_WARNING
#warning You have defined that you want WiFi but your hardware has not enough memory to do that, so WiFi DISABLED
#endif
#ifdef ETHERNET_WARNING
#warning You have defined that you want Ethernet but your hardware has not enough memory to do that, so Ethernet DISABLED
#endif
#ifdef EXRAIL_WARNING
#warning You have myAutomation.h but your hardware has not enough memory to do that, so EX-RAIL DISABLED
#endif


#define PASSWDCHECK(S) static_assert(sizeof(S) == 1 || sizeof(S) > 8, "Password shorter than 8 chars")
void setup();
void loop();
#line 80 "C:/GitHub/CommandStation-EX/CommandStation-EX.ino"
void setup()
{




  SerialManager::init();

  DIAG(F("License GPLv3 fsf.org (c) dcc-ex.com"));


#if defined(STARTUP_DELAY)
  DIAG(F("Delaying startup for %dms"), STARTUP_DELAY);
  delay(STARTUP_DELAY);
#endif


  IODevice::begin();



  ADCee::begin();

  TrackManager::Setup(MOTOR_SHIELD_TYPE);

  DISPLAY_START (

    LCD(0,F("DCC-EX v" VERSION));
    LCD(1,F("Lic GPLv3"));
  );




#if WIFI_ON
  PASSWDCHECK(WIFI_PASSWORD);
#ifndef ARDUINO_ARCH_ESP32
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#else
  WifiESP::setup();
#endif
#endif

#if ETHERNET_ON
  EthernetInterface::setup();
#endif


  DCC::begin();


  RMFT::begin();

#ifdef ARDUINO_ARCH_ESP32
#ifdef BOOSTER_INPUT
  dccSniffer = new Sniffer(BOOSTER_INPUT);
#endif
#endif



  #if __has_include ( "mySetup.h")
    #define SETUP(cmd) DCCEXParser::parse(F(cmd))
    #include "mySetup.h"
    #undef SETUP
  #endif

  #if defined(LCN_SERIAL)
  LCN_SERIAL.begin(115200);
  LCN::init(LCN_SERIAL);
  #endif
  LCD(3, F("Ready"));
  CommandDistributor::broadcastPower();
}

void loop()
{
  #ifdef ENABLE_SERIAL_LOG
    SerialLog.loop();
  #endif

#ifdef ARDUINO_ARCH_ESP32

#ifdef BOOSTER_INPUT
  static bool oldactive = false;
  if (dccSniffer) {
    bool newactive = dccSniffer->inputActive();
    if (oldactive != newactive) {
      RMFT2::railsyncEvent(newactive);
      oldactive = newactive;
    }
    DCCPacket p = dccSniffer->fetchPacket();
    if (p.len() != 0) {
      if (DCCDecoder::parse(p)) {
 if (Diag::SNIFFER)
   p.print();
      }
    }
  }
#endif
#endif





  DCC::loop();


  SerialManager::loop();


#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::loop();

#endif
#else
#if WIFI_ON
#ifndef WIFI_TASK_ON_CORE0
  WifiESP::loop();
#endif
#endif
#endif
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();

  #if defined(LCN_SERIAL)
  LCN::loop();
  #endif


  DisplayInterface::loop();


  IODevice::loop();

  Sensor::checkAll();


  static int ramLowWatermark = __INT_MAX__;

  #ifdef ARDUINO_ARCH_AVR

  int freeNow = DCCTimer::getMinimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5db"), ramLowWatermark);
  }
  #else

  int freeNow = DCCTimer::getMinimumFreeMemory() / 4096;
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5dKb"), ramLowWatermark*4);
  }
  #endif
}