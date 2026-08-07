#include "defines.h"

#if defined(ARDUINO_ARCH_ESP32) && ETHERNET_ON == true
#include <SPI.h>

// Called by the ESP32 Arduino core during initArduino().
// Pre-start SPI with Ethernet pins so later SPI.begin() calls in libraries
// keep this bus instance and do not remap to board defaults.
extern "C" void initVariant(void) {
  SPI.begin(ETHERNET_SCK_PIN, ETHERNET_MISO_PIN, ETHERNET_MOSI_PIN, ETHERNET_CS_PIN);
}

#endif