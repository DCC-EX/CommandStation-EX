// EXIO_TCPClientProvider.cpp
#include "EXIO_TCPClientProvider.h"
#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
#endif

#if ETHERNET_ON == true
  // Pulls in the right Ethernet stack for the board (NativeEthernet/STM32Ethernet/Ethernet)
  #include "EthernetInterface.h"
#endif

namespace EXIO_TCPClientProvider {

  bool networkReady() {
  #if defined(ARDUINO_ARCH_ESP32)
    // In AP mode, WL_CONNECTED is not guaranteed, so just ensure WiFi isn't off.
    return WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA;
  #elif ETHERNET_ON == true
    // If there is no cable, link will be off; still safe to return false.
    return Ethernet.linkStatus() != LinkOFF;
  #else
    // ESP-AT (WifiInterface) is Stream/AT based and does NOT expose an Arduino Client.
    return false;
  #endif
  }

  Client* createClient() {
  #if defined(ARDUINO_ARCH_ESP32)
    return new WiFiClient();
  #elif ETHERNET_ON == true
    return new EthernetClient();
  #else
    // No Client-based TCP stack in this build.
    // (WifiInterface / ESP-AT uses AT commands over Stream, not Client.)
    return nullptr;
  #endif
  }

}