/*
 *  © 2026 Herb Morton
 *  © 2026 Mathew Winters
 *
 *  All rights reserved.
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

#ifndef EthernetPins_h
#define EthernetPins_h

#define _ETH_STR_HELPER(x) #x
#define _ETH_STR(x)        _ETH_STR_HELPER(x)

#define CSB1_SPI         1
#define ESP32_SPI        2
#define ESP32_SPI2       3
#define NUCLEO64_SPI1    4
#define NUCLEO64_SPI2    5
#define NUCLEO144_SPI2   6
#define NUCLEO144_SPI2B  7
#define NUCLEO144_F4X9   8

#if defined(ETHERNET_PINS)
  #ifndef ETHERNET_NAME
    #define ETHERNET_NAME _ETH_STR(ETHERNET_PINS)
  #endif

  #if ETHERNET_PINS == CSB1_SPI
    #define ETHERNET_PNAME     "CSB1_SPI"  // uses Track C pins
    #define ETHERNET_CS_PIN   5
    #define ETHERNET_SCK_PIN  18
    #define ETHERNET_MISO_PIN 36
    #define ETHERNET_MOSI_PIN 26
    #define ETHERNET_RST_PIN  13
  #elif ETHERNET_PINS == ESP32_SPI
    #define ETHERNET_PNAME     "ESP32_SPI"
    #define ETHERNET_CS_PIN   5
    #define ETHERNET_SCK_PIN  18
    #define ETHERNET_MISO_PIN 19
    #define ETHERNET_MOSI_PIN 23
    #define ETHERNET_RST_PIN  4
  #elif ETHERNET_PINS == ESP32_SPI2
    #define ETHERNET_PNAME     "ESP32_SP2"
    #define ETHERNET_CS_PIN   15
    #define ETHERNET_SCK_PIN  14
    #define ETHERNET_MISO_PIN 12
    #define ETHERNET_MOSI_PIN 13
    #define ETHERNET_RST_PIN  4
  #elif ETHERNET_PINS == NUCLEO64_SPI1
    #define ETHERNET_PNAME     "NUCLEO64_SPI1"
    #define ETHERNET_CS_PIN   D10
    #define ETHERNET_SCK_PIN  D13
    #define ETHERNET_MISO_PIN D12
    #define ETHERNET_MOSI_PIN D11
  #elif ETHERNET_PINS == NUCLEO64_SPI2
    #define ETHERNET_PNAME     "NUCLEO64_SPI2"
    #define ETHERNET_CS_PIN   PB12
    #define ETHERNET_SCK_PIN  PB13
    #define ETHERNET_MISO_PIN PB14
    #define ETHERNET_MOSI_PIN PB15
  #elif ETHERNET_PINS == NUCLEO144_SPI2
    #define ETHERNET_PNAME     "NUCLEO144_SPI2"
    #define ETHERNET_CS_PIN   PB12
    #define ETHERNET_SCK_PIN  PB13
    #define ETHERNET_MISO_PIN PB14
    #define ETHERNET_MOSI_PIN PB15
  #elif ETHERNET_PINS == NUCLEO144_SPI2B
    #define ETHERNET_PNAME     "NUCLEO144_SPI2B"
    #define ETHERNET_CS_PIN   PB12
    #define ETHERNET_SCK_PIN  PD3
    #define ETHERNET_MISO_PIN PC2
    #define ETHERNET_MOSI_PIN PB15
  #elif ETHERNET_PINS == NUCLEO144_F4X9
    #define ETHERNET_PNAME     "NUCLEO144_F4X9"
    // Native RMII MAC on F429ZI/F439ZI; ETHERNET_CS_PIN intentionally omitted
  #else
    #error "Unknown ETHERNET_PINS preset selected"
  #endif
#endif

#endif
