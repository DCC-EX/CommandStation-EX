/*
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2022 Chris Harlow
 *  © 2023 Nathan Kellenicki
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
#ifndef ARDUINO_AVR_UNO_WIFI_REV2
// This code is NOT compiled on a unoWifiRev2 processor which uses a different architecture 
#include "WifiInterface.h"        /* config.h included there */
//#include <avr/pgmspace.h>
#include "DIAG.h"
#include "StringFormatter.h"

#include "WifiInboundHandler.h"




const unsigned long LOOP_TIMEOUT = 2000;
bool WifiInterface::connected = false;
Stream * WifiInterface::wifiStream;

#ifndef WIFI_CONNECT_TIMEOUT
// Tested how long it takes to FAIL an unknown SSID on firmware 1.7.4.
// The ES should fail a connect in 15 seconds, we don't want to fail BEFORE that
// or ot will cause issues with the following commands. 
#define WIFI_CONNECT_TIMEOUT 16000
#endif

////////////////////////////////////////////////////////////////////////////////
//
// Figure out number of serial ports depending on hardware
//
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
#define NUM_SERIAL 0
#endif
 
#if (defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560))
#define NUM_SERIAL 3
#define SERIAL1 Serial1
#define SERIAL3 Serial3
#endif

#if defined(ARDUINO_ARCH_STM32)
// Handle serial ports availability on STM32 for variants!
// #undef NUM_SERIAL
#if defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE)
#define NUM_SERIAL 3
#define SERIAL1 Serial1
#define SERIAL3 Serial6
#elif defined(ARDUINO_NUCLEO_F446RE)
#define NUM_SERIAL 3
#define SERIAL1 Serial3
#define SERIAL3 Serial5
#elif defined(ARDUINO_NUCLEO_F413ZH) || defined(ARDUINO_NUCLEO_F429ZI) \
    || defined(ARDUINO_NUCLEO_F446ZE) || defined(ARDUINO_NUCLEO_F412ZG) \
    || defined(ARDUINO_NUCLEO_F439ZI)
#define NUM_SERIAL 2
#define SERIAL1 Serial6
#else
#warning This variant of Nucleo not yet explicitly supported
#endif
#endif

#ifndef NUM_SERIAL
#define NUM_SERIAL 1
#define SERIAL1 Serial1
#endif

bool WifiInterface::setup(long serial_link_speed, 
                          const FSH *wifiESSID,
                          const FSH *wifiPassword,
                          const FSH *hostname,
                          const int port,
                          const byte channel,
                          const bool forceAP) {

  wifiSerialState wifiUp = WIFI_NOAT;

#if NUM_SERIAL == 0
  // no warning about unused parameters. 
  (void) serial_link_speed;
  (void) wifiESSID;
  (void) wifiPassword;
  (void) hostname;
  (void) port;
  (void) channel;
  (void) forceAP;
#endif  

// See if the WiFi is attached to the first serial port
#if NUM_SERIAL > 0 && !defined(SERIAL1_COMMANDS)
  SERIAL1.begin(serial_link_speed);
  wifiUp = setup(SERIAL1, wifiESSID, wifiPassword, hostname, port, channel, forceAP);
#endif

// Other serials are tried, depending on hardware.
// Currently only the Arduino Mega 2560 has usable Serial2 (Nucleo-64 boards use Serial 2 for console!)
#if defined(ARDUINO_AVR_MEGA2560)
#if NUM_SERIAL > 1 && !defined(SERIAL2_COMMANDS)
  if (wifiUp == WIFI_NOAT)
  {
    Serial2.begin(serial_link_speed);
    wifiUp = setup(Serial2, wifiESSID, wifiPassword, hostname, port, channel, forceAP);
  }
#endif
#endif

// We guess here that in all architctures that have a Serial3
// we can use it for our purpose.
#if NUM_SERIAL > 2 && !defined(SERIAL3_COMMANDS)
  if (wifiUp == WIFI_NOAT)
  {
    SERIAL3.begin(serial_link_speed);
    wifiUp = setup(SERIAL3, wifiESSID, wifiPassword, hostname, port, channel, forceAP);
  }
#endif

  if (wifiUp == WIFI_NOAT) // here and still not AT commands found
      return false;

  DCCEXParser::setAtCommandCallback(ATCommand);
  // CAUTION... ONLY CALL THIS ONCE 
  WifiInboundHandler::setup(wifiStream);
  if (wifiUp == WIFI_CONNECTED)
      connected = true;
  else
      connected = false;
  return connected; 
}

wifiSerialState WifiInterface::setup(Stream & setupStream,  const FSH* SSid, const FSH* password,
				     const FSH* hostname,  int port, byte channel, bool forceAP) {
  wifiSerialState wifiState;
  static uint8_t ntry = 0;
  ntry++;

  wifiStream = &setupStream;

  DIAG(F("++ Wifi Setup Try %d ++"), ntry);

  wifiState = setup2( SSid, password, hostname,  port, channel, forceAP);

  if (wifiState == WIFI_NOAT) {
    LCD(4, F("WiFi no AT chip"));
    return wifiState;
  }
 
  if (wifiState == WIFI_CONNECTED) {
    StringFormatter::send(wifiStream, F("ATE0\r\n")); // turn off the echo 
    checkForOK(200, true);
    DIAG(F("WiFi CONNECTED"));
    // LCD already shows IP
  } else {
    LCD(4,F("WiFi DISCON."));
  }
  return wifiState;
}

#ifdef DONT_TOUCH_WIFI_CONF
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
wifiSerialState WifiInterface::setup2(const FSH* SSid, const FSH* password,
				      const FSH* hostname, int port, byte channel, bool forceAP) {
  bool ipOK = false;
  bool oldCmd = false;

  char macAddress[17];  //  mac address extraction   
 
  // First check... Restarting the Arduino does not restart the ES. 
  //  There may alrerady be a connection with data in the pipeline.
  // If there is, just shortcut the setup and continue to read the data as normal.
  if (checkForOK(200,F("+IPD"), true)) {
    DIAG(F("Preconfigured Wifi already running with data waiting"));
    return WIFI_CONNECTED; 
  }

  StringFormatter::send(wifiStream, F("AT\r\n"));   // Is something here that understands AT?
  if(!checkForOK(200, true))
    return WIFI_NOAT;                               // No AT compatible WiFi module here

  StringFormatter::send(wifiStream, F("ATE1\r\n")); // Turn on the echo, se we can see what's happening
  checkForOK(2000, true);                // Makes this visible on the console

  // Display the AT version information
  StringFormatter::send(wifiStream, F("AT+GMR\r\n")); 
  if (checkForOK(2000, F("AT version:"), true, false)) {
    char version[] = "0.0.0.0-xxx";
    for (int i=0; i<11;i++) {
      while(!wifiStream->available());
      version[i]=wifiStream->read();
      StringFormatter::printEscape(version[i]);
    }
    if ((version[0] == '0') ||
	(version[0] == '2' && version[2] == '0') ||
	(version[0] == '2' && version[2] == '2' && version[4] == '0' && version[6] == '0'
	 && version[7] == '-' && version[8] == 'd' && version[9] == 'e' && version[10] == 'v')) {
      DIAG(F("You need to up/downgrade the ESP firmware"));
      SSid = F("UPDATE_ESP_FIRMWARE");
      forceAP = true;
    }
  }
  checkForOK(2000, true, false);

#ifdef DONT_TOUCH_WIFI_CONF
  DIAG(F("DONT_TOUCH_WIFI_CONF was set: Using existing config"));
#else
  // Older ES versions have AT+CWJAP, newer ones have AT+CWJAP_CUR and AT+CWHOSTNAME
  StringFormatter::send(wifiStream, F("AT+CWJAP_CUR?\r\n"));
  if (!(checkForOK(2000, true))) {
      oldCmd=true;
      while (wifiStream->available()) StringFormatter::printEscape( wifiStream->read()); /// THIS IS A DIAG IN DISGUISE
  }

  StringFormatter::send(wifiStream, F("AT+CWMODE%s=1\r\n"), oldCmd ? "" : "_CUR"); // configure as "station" = WiFi client
  checkForOK(1000, true);                       // Not always OK, sometimes "no change"

  const char *yourNetwork = "Your network ";
  if (STRNCMP_P(yourNetwork, (const char*)SSid, 13) == 0 || STRNCMP_P("", (const char*)SSid, 13) == 0) {
    if (STRNCMP_P(yourNetwork, (const char*)password, 13) == 0) {
      // If the source code looks unconfigured, check if the
      // ESP8266 is preconfigured in station mode.
      // We check the first 13 chars of the SSid and the password

      // give a preconfigured ES8266 a chance to connect to a router
      // typical connect time approx 7 seconds
      delay(8000);
      StringFormatter::send(wifiStream, F("AT+CIFSR\r\n"));
      if (checkForOK(5000, F("+CIFSR:STAIP"), true,false))
	  if (!checkForOK(1000, F("0.0.0.0"), true,false))
	      ipOK = true;
    }
  } else if (!forceAP) {
      // SSID was configured, so we assume station (client) mode.
      if (oldCmd) {
	      // AT command early version supports CWJAP/CWSAP
	      StringFormatter::send(wifiStream, F("AT+CWJAP=\"%S\",\"%S\"\r\n"), SSid, password);
	      ipOK = checkForOK(WIFI_CONNECT_TIMEOUT, true);
      } else {
      // later version supports CWJAP_CUR
        StringFormatter::send(wifiStream, F("AT+CWHOSTNAME=\"%S\"\r\n"), hostname); // Set Host name for Wifi Client
      	checkForOK(2000, true); // dont care if not supported
      
        StringFormatter::send(wifiStream, F("AT+CWJAP_CUR=\"%S\",\"%S\"\r\n"), SSid, password);
        ipOK = checkForOK(WIFI_CONNECT_TIMEOUT, true);
      }

      if (ipOK) {
	      // But we really only have the ESSID and password correct
        // Let's check for IP (via DHCP)
        ipOK = false;
        StringFormatter::send(wifiStream, F("AT+CIFSR\r\n"));
        if (checkForOK(5000, F("+CIFSR:STAIP"), true,false))
        if (!checkForOK(1000, F("0.0.0.0"), true,false))
        ipOK = true;
      }
  }

  if (!ipOK) {
    // If we have not managed to get this going in station mode, go for AP mode

  //    StringFormatter::send(wifiStream, F("AT+RST\r\n"));
  //    checkForOK(1000, true); // Not always OK, sometimes "no change"

    int i=0;
    do {
      // configure as AccessPoint. Try really hard as this is the
      // last way out to get any Wifi connectivity. 
      StringFormatter::send(wifiStream, F("AT+CWMODE%s=2\r\n"), oldCmd ? "" : "_CUR"); 
    } while (!checkForOK(1000+i*500, true) && i++<10);

    while (wifiStream->available()) StringFormatter::printEscape( wifiStream->read()); /// THIS IS A DIAG IN DISGUISE

    // Figure out MAC addr
    StringFormatter::send(wifiStream, F("AT+CIFSR\r\n")); // not TOMATO
    // looking fpr mac addr eg +CIFSR:APMAC,"be:dd:c2:5c:6b:b7"
    if (checkForOK(5000, F("+CIFSR:APMAC,\""), true,false)) {
      // Copy 17 byte mac address
      for (int i=0; i<17;i++) {
        while(!wifiStream->available());
	macAddress[i]=wifiStream->read();
	StringFormatter::printEscape(macAddress[i]);
      }
    } else {
	memset(macAddress,'f',sizeof(macAddress));
    }
    char macTail[]={macAddress[9],macAddress[10],macAddress[12],macAddress[13],macAddress[15],macAddress[16],'\0'};

    checkForOK(1000, true, false);  // suck up remainder of AT+CIFSR
  
    i=0;
    do {
      if (!forceAP) {
        if (STRNCMP_P(yourNetwork, (const char*)password, 13) == 0) {
    // unconfigured
          StringFormatter::send(wifiStream, F("AT+CWSAP%s=\"DCCEX_%s\",\"PASS_%s\",%d,4\r\n"),
                                            oldCmd ? "" : "_CUR", macTail, macTail, channel);
        } else {
          // password configured by user
          StringFormatter::send(wifiStream, F("AT+CWSAP%s=\"DCCEX_%s\",\"%S\",%d,4\r\n"), oldCmd ? "" : "_CUR",
                                          macTail, password, channel);
        }
      } else {
        StringFormatter::send(wifiStream, F("AT+CWSAP%s=\"%S\",\"%S\",%d,4\r\n"),
                                        oldCmd ? "" : "_CUR", SSid, password, channel);
      }
    } while (!checkForOK(WIFI_CONNECT_TIMEOUT, true) && i++<2); // do twice if necessary but ignore failure as AP mode may still be ok
    if (i >= 2)
	DIAG(F("Warning: Setting AP SSID and password failed"));       // but issue warning

    if (!oldCmd) {
      StringFormatter::send(wifiStream, F("AT+CIPRECVMODE=0\r\n"), port); // make sure transfer mode is correct
      checkForOK(2000, true);
    }
  }
#endif //DONT_TOUCH_WIFI_CONF

  StringFormatter::send(wifiStream, F("AT+CIPSERVER=0\r\n")); // turn off tcp server (to clean connections before CIPMUX=1)
  checkForOK(1000, true); // ignore result in case it already was off

  StringFormatter::send(wifiStream, F("AT+CIPMUX=1\r\n")); // configure for multiple connections
  if (!checkForOK(1000, true)) return WIFI_DISCONNECTED;

  if(!oldCmd) {                                                                    // no idea to test this on old firmware
    StringFormatter::send(wifiStream, F("AT+MDNS=1,\"%S\",\"withrottle\",%d\r\n"),
			  hostname, port);                                         // mDNS responder
    checkForOK(1000, true);                                                        // dont care if not supported
  }

  StringFormatter::send(wifiStream, F("AT+CIPSERVER=1,%d\r\n"), port); // turn on server on port
  if (!checkForOK(1000, true)) return WIFI_DISCONNECTED;
 
  StringFormatter::send(wifiStream, F("AT+CIFSR\r\n")); // Display  ip addresses to the DIAG 
  if (!checkForOK(1000, F("IP,\"") , true, false)) return WIFI_DISCONNECTED;
  // Copy the IP address
  {
    const byte MAX_IP_LENGTH=15;
    char ipString[MAX_IP_LENGTH+1];
    ipString[MAX_IP_LENGTH]='\0'; // protection against missing " character on end. 
    for(byte ipLen=0;ipLen<MAX_IP_LENGTH;ipLen++) {
      while(!wifiStream->available());
      int ipChar=wifiStream->read();
      StringFormatter::printEscape(ipChar);
      if (ipChar=='"') {
        ipString[ipLen]='\0';
        break;
      }
      ipString[ipLen]=ipChar;
    }
    LCD(4,F("%s"),ipString);  // There is not enough room on some LCDs to put a title to this      
  }
  // suck up anything after the IP. 
  if (!checkForOK(1000, true, false)) return WIFI_DISCONNECTED;
  LCD(5,F("PORT=%d"),port);
   
  return WIFI_CONNECTED;
}
#ifdef DONT_TOUCH_WIFI_CONF
#pragma GCC diagnostic pop
#endif


// This function is used to allow users to enter <+ commands> through the DCCEXParser
// <+command>  sends AT+command to the ES and returns to the caller.
// Once the user has made whatever changes to the AT commands, a <+X> command can be used
// to force on the connectd flag so that the loop will start picking up wifi traffic.
// If the settings are corrupted <+RST> will clear this and then you must restart the arduino.

// Using the <+> command with no command string causes the code to enter an echo loop so that all
// input is directed to the ES and all ES output written to the USB Serial.
// The sequence "!!!" returns the Arduino to the normal loop mode

 
void WifiInterface::ATCommand(HardwareSerial * stream,const byte * command) {
  command++;
  if (*command=='\0') { // User gave <+> command  
    stream->print(F("\nES AT command passthrough mode, use ! to exit\n"));
    while(stream->available()) stream->read(); // Drain serial input first 
    bool startOfLine=true;
    while(true) {
      while (wifiStream->available()) stream->write(wifiStream->read());
      if (stream->available()) {
        int cx=stream->read();
        // A newline followed by ! is an exit
        if (cx=='\n' || cx=='\r') startOfLine=true; 
        else if (startOfLine && cx=='!')  break;
        else startOfLine=false; 
        wifiStream->write(cx);  
      }
    }
    stream->print(F("Passthrough Ended"));
    return; 
  }
  
  if (*command=='X') {
    connected = true;
    DIAG(F("++++++ Wifi Connction forced on ++++++++"));
  }
  else {
    StringFormatter::  send(wifiStream, F("AT+%s\r\n"), command);
    checkForOK(10000,  true);
  }
}



bool WifiInterface::checkForOK( const unsigned int timeout,  bool echo, bool escapeEcho) {
  return checkForOK(timeout,F("\r\nOK\r\n"),echo,escapeEcho);
}

bool WifiInterface::checkForOK( const unsigned int timeout, const FSH * waitfor, bool echo, bool escapeEcho) {
  unsigned long  startTime = millis();
  char *locator = (char *)waitfor;
  DIAG(F("Wifi Check: [%E]"), waitfor);
  while ( millis() - startTime < timeout) {
    int nextchar;
    while (wifiStream->available() && (nextchar = wifiStream->read()) > -1) {
      char ch = (char)nextchar;
      if (echo) {
        if (escapeEcho) StringFormatter::printEscape( ch); /// THIS IS A DIAG IN DISGUISE
        else USB_SERIAL.print(ch);
      }
      if (ch != GETFLASH(locator)) locator = (char *)waitfor;
      if (ch == GETFLASH(locator)) {
        locator++;
        if (!GETFLASH(locator)) {
          DIAG(F("Found in %dms"), millis() - startTime);
          return true;
        }
      }
    }
  }
  DIAG(F("TIMEOUT after %dms"), timeout);
  return false;
}


void WifiInterface::loop() {
  if (connected) {
    WifiInboundHandler::loop(); 
  }
}

#endif
