/*
 *  WiFiInterface.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "WiFiInterface.h"
#include "../Utils/DIAG.h"
#include "../CommInterface/CommManager.h"
#include "WiThrottle.h"

const char PROGMEM READY_SEARCH[]  = "\r\nready\r\n";
const char PROGMEM OK_SEARCH[] = "\r\nOK\r\n";
const char PROGMEM END_DETAIL_SEARCH[] = "@ 1000";
const char PROGMEM PROMPT_SEARCH[] = ">";
const char PROGMEM SEND_OK_SEARCH[] = "\r\nSEND OK\r\n";
const char PROGMEM IPD_SEARCH[] = "+IPD";
const unsigned long LOOP_TIMEOUT = 2000;

bool WiFiInterface::connected = false;
bool WiFiInterface::closeAfter = false;
byte WiFiInterface::loopstate = 0;
unsigned long WiFiInterface::loopTimeoutStart = 0;
int WiFiInterface::datalength = 0;
int WiFiInterface::connectionId;
byte WiFiInterface::buffer[MAX_WIFI_BUFFER];
MemStream  WiFiInterface::streamer(buffer, sizeof(buffer));
Stream * WiFiInterface::wifiStream = NULL;
HTTP_CALLBACK WiFiInterface::httpCallback = 0;

void WiFiInterface::setup(Stream * setupStream,  const __FlashStringHelper* ssid, const __FlashStringHelper* password,
              const __FlashStringHelper* hostname, const __FlashStringHelper* servername, int port) {

  wifiStream = setupStream;

  DIAG(F("\n\rWifi Setup...\n\r"));

  connected = setup2( ssid, password, hostname, servername, port);
 
  if (connected) CommManager::send(wifiStream, F("ATE0\r\n")); // turn off the echo server on port
 
  DIAG(F("Wifi %S \n\r"), connected ? F("OK") : F("FAILED"));
}

bool WiFiInterface::setup2(const __FlashStringHelper* ssid, const __FlashStringHelper* password,
               const __FlashStringHelper* hostname, const __FlashStringHelper* servername, int port) {
  
  int ipOK = 0;

  if (checkForOK(200, IPD_SEARCH, true)) {
    DIAG(F("Prev. Wifi found, data waiting\n\r"));
    loopstate=4;  // carry on from correct place 
    return true; 
  }

  CommManager::send(wifiStream, F("ATE1\r\n")); // turn on the echo server on port
 
  CommManager::send(wifiStream, F("AT+GMR\r\n")); // request AT  version
  checkForOK(2000, OK_SEARCH, true, false);      // Makes this visible on the console

  CommManager::send(wifiStream, F("AT+CWMODE=1\r\n")); // configure as station
  checkForOK(1000, OK_SEARCH, true); // Not always OK, sometimes "no change"
  
  delay(8000); // give preconfigured ESP8266 a chance to connect 
  
  CommManager::send(wifiStream, F("AT+CIFSR\r\n"));
  if (checkForOK(5000, (const char*) F("+CIFSR:STAIP"), true, false))
    if (!checkForOK(1000, (const char*) F("0.0.0.0"), true,false))
      ipOK = 1;

  if (!ipOK) {
    CommManager::send(wifiStream, F("AT+CWMODE=3\r\n")); // configure as server or access point
    checkForOK(1000, OK_SEARCH, true); // Not always OK, sometimes "no change"

    // Older ES versions have AT+CWJAP, newer ones have AT+CWJAP_CUR and AT+CWHOSTNAME
    CommManager::send(wifiStream, F("AT+CWJAP?\r\n"));
    if (checkForOK(2000, OK_SEARCH, true)) {
      // early version supports CWJAP
      DIAG(F("CWJAP"));
      CommManager::send(wifiStream, F("AT+CWJAP=\"%S\",\"%S\"\r\n"), ssid, password);
      checkForOK(20000, OK_SEARCH, true); // can ignore failure as AP mode may still be ok
    }
    else {
      // later version supports CWJAP_CUR
      CommManager::send(wifiStream, F("AT+CWHOSTNAME=\"%S\"\r\n"), hostname); // Set Host name for Wifi Client
      checkForOK(2000, OK_SEARCH, true); // dont care if not supported

      DIAG(F("CWJAP_CUR"));
      CommManager::send(wifiStream, F("AT+CWJAP_CUR=\"%S\",\"%S\"\r\n"), ssid, password);
      checkForOK(20000, OK_SEARCH, true); // can ignore failure as AP mode may still be ok

      CommManager::send(wifiStream, F("AT+CIPRECVMODE=0\r\n"), port); // make sure transfer mode is correct
      checkForOK(2000, OK_SEARCH, true);

      // StringFormatter::send(wifiStream, F("AT+MDNS=1,\"%S.local\",\"%S.local\",%d\r\n"), hostname, servername, port); // Setup mDNS for Server
      // if (!checkForOK(5000, OK_SEARCH, true)) return false;

      (void)servername; // avoid compiler warning from commented out AT_MDNS above
    }
    CommManager::send(wifiStream, F("AT+CIFSR\r\n")); // get ip address
    if (!checkForOK(10000, OK_SEARCH, true, false)) return false;
  }

  CommManager::send(wifiStream, F("AT+CIPMUX=1\r\n")); // configure for multiple connections
  if (!checkForOK(10000, OK_SEARCH, true)) return false;

  CommManager::send(wifiStream, F("AT+CIPSERVER=1,%d\r\n"), port); // turn on server on port
  if (!checkForOK(10000, OK_SEARCH, true)) return false;

  return true;
}


// This function is used to allow users to enter <+ commands> through the DCCEXParser
// Once the user has made whatever changes to the AT commands, a <+X> command can be used
// to force on the connectd flag so that the loop will start picking up wifi traffic.
// If the settings are corrupted <+RST> will clear this and then you must restart the arduino.
void WiFiInterface::ATCommand(const char * command) {
  if (*command=='X') {
    connected = true;
    DIAG(F("Wifi forced conn.\n\r"));
  }
  else {
    CommManager::send(wifiStream, F("AT+%s\r\n"), command + 1);
    checkForOK(10000, OK_SEARCH, true);
  }
}

void WiFiInterface::setHTTPCallback(HTTP_CALLBACK callback) {
  httpCallback = callback;
}

bool WiFiInterface::checkForOK( const unsigned int timeout, const char * waitfor, bool echo, bool escapeEcho) {
  unsigned long  startTime = millis();
  char  const *locator = waitfor;
  DIAG(F("\n\rWifi Check: [%E]"), waitfor);
  while ( millis() - startTime < timeout) {
    while (wifiStream->available()) {
      int ch = wifiStream->read();
      if (echo) {
        if (escapeEcho) CommManager::printEscape(&DIAGSERIAL, ch); /// THIS IS A DIAG IN DISGUISE
        else DIAG(F("%c"), ch); 
      }
      if (ch != pgm_read_byte_near(locator)) locator = waitfor;
      if (ch == pgm_read_byte_near(locator)) {
        locator++;
        if (!pgm_read_byte_near(locator)) {
          DIAG(F("\n\rFound in %dms"), millis() - startTime);
          return true;
        }
      }
    }
  }
  DIAG(F("\n\rTIMEOUT after %dms\n\r"), timeout);
  return false;
}

bool WiFiInterface::isHTTP() {
  // POST GET PUT PATCH DELETE
  // You may think a simple strstr() is better... but not when ram & time is in short supply
  switch (buffer[0]) {
  case 'P':
    if (buffer[1] == 'U' && buffer[2] == 'T' && buffer[3] == ' ' ) return true;
    if (buffer[1] == 'O' && buffer[2] == 'S' && buffer[3] == 'T' && buffer[4] == ' ') return true;
    if (buffer[1] == 'A' && buffer[2] == 'T' && buffer[3] == 'C' && buffer[4] == 'H' && buffer[5] == ' ') return true;
    return false;
  case 'G':
    if (buffer[1] == 'E' && buffer[2] == 'T' && buffer[3] == ' ' ) return true;
    return false;
  case 'D':
    if (buffer[1] == 'E' && buffer[2] == 'L' && buffer[3] == 'E' && buffer[4] == 'T' && buffer[5] == 'E' && buffer[6] == ' ') return true;
    return false;
  default:
    return false;
  }
}

void WiFiInterface::loop() {
  if (!connected) return;

  WiThrottle::loop();  // check heartbeats

  // read anything into a buffer, collecting info on the way
  while (loopstate != 99 && wifiStream->available()) {
    int ch = wifiStream->read();

    // echo the char to the diagnostic stream in escaped format
    CommManager::printEscape(&DIAGSERIAL, ch); // DIAG in disguise

    switch (loopstate) {
    case 0:  // looking for +IPD
      connectionId = 0;
      if (ch == '+') loopstate = 1;
      break;
    case 1:  // Looking for I   in +IPD
      loopstate = (ch == 'I') ? 2 : 0;
      break;
    case 2:  // Looking for P   in +IPD
      loopstate = (ch == 'P') ? 3 : 0;
      break;
    case 3:  // Looking for D   in +IPD
      loopstate = (ch == 'D') ? 4 : 0;
      break;
    case 4:  // Looking for ,   After +IPD
      loopstate = (ch == ',') ? 5 : 0;
      break;
    case 5:  // reading connection id
      if (ch == ',') loopstate = 6;
      else connectionId = 10 * connectionId + (ch - '0');
      break;
    case 6: // reading for length
      if (ch == ':') loopstate = (datalength == 0) ? 99 : 7; // 99 is getout without reading next char
      else datalength = datalength * 10 + (ch - '0');
      streamer.flush();  // basically sets write point at start of buffer
      break;
    case 7: // reading data
      streamer.write(ch);
      datalength--;
      if (datalength == 0) loopstate = 99;
      break;

    case 10:  // Waiting for > so we can send reply
      if (millis() - loopTimeoutStart > LOOP_TIMEOUT) {
        DIAG(F("\n\rWifi TIMEOUT on wait for > prompt or ERROR\n\r"));
        loopstate = 0; // go back to +IPD
        break;
      }
      if (ch == '>') {
        //                  DIAG(F("\n\r> [%e]\n\r"),buffer);
        wifiStream->print((char *) buffer);
        loopTimeoutStart = millis();
        loopstate = closeAfter ? 11 : 0;
        break;
      }
      if (ch == '.') { // busy during send, delay and retry  
        loopstate = 12; // look for SEND OK finished 
        break;
      }
      break;
    case 11: // Waiting for SEND OK or ERROR to complete so we can closeAfter
      if (millis() - loopTimeoutStart > LOOP_TIMEOUT) {
        DIAG(F("\n\rWifi TIMEOUT on wait for SEND OK or ERROR\n\r"));
        loopstate = 0; // go back to +IPD
        break;
      }
      if (ch == 'K') { // assume its in  SEND OK
        DIAG(F("\n\r Wifi AT+CIPCLOSE=%d\r\n"), connectionId);
        CommManager::send(wifiStream, F("AT+CIPCLOSE=%d\r\n"), connectionId);
        loopstate = 0; // wait for +IPD
      }
      break;

    case 12: // Waiting for OK after send busy 
      if (ch == '+') { // Uh-oh IPD problem
        DIAG(F("\n\n Wifi ASYNC CLASH - LOST REPLY\n\r"));
        connectionId = 0;
        loopstate = 1;
      }
      if (ch == 'K') { // assume its in  SEND OK
        DIAG(F("\n\n Wifi BUSY RETRYING.. AT+CIPSEND=%d,%d\r\n"), connectionId, streamer.available() - 1);
        CommManager::send(wifiStream, F("AT+CIPSEND=%d,%d\r\n"), connectionId, streamer.available() - 1);
        loopTimeoutStart = millis();
        loopstate = 10; // non-blocking loop waits for > before sending
        break;
      }
      break;
    } // switch
  } // while
  if (loopstate != 99) return;

  // AT this point we have read an incoming message into the buffer
  streamer.print('\0'); // null the end of the buffer so we can treat it as a string

  DIAG(F("\n%l Wifi(%d)<-[%e]\n"), millis(), connectionId, buffer);
  streamer.setBufferContentPosition(0, 0); // reset write position to start of buffer
  // SIDE EFFECT WARNING:::
  //  We know that parser will read the entire buffer before starting to write to it.
  //  Otherwise we would have to copy the buffer elsewhere and RAM is in short supply.

  closeAfter = false;

  // Intercept HTTP requests
  if (isHTTP()) {
    if (httpCallback) httpCallback(&streamer, buffer);
    else {
      CommManager::send(&streamer, F("HTTP/1.1 404 Not Found\nContent-Type: text/html\nConnection: close\n\n"));
      CommManager::send(&streamer, F("<html><body>This is <b>not</b> a web server.<br/></body></html>"));
    }
    closeAfter = true;
  }
  else if (buffer[0] == '<') DCCEXParser::parse(&streamer, (const char*) buffer); // tell JMRI parser that ACKS are blocking because we can't handle the async
  else WiThrottle::getThrottle(connectionId)->parse(&streamer, buffer);

  if (streamer.available() == 0) {
    // No reply
    if (closeAfter) {
      DIAG(F("AT+CIPCLOSE=%d\r\n"), connectionId);
      CommManager::send(wifiStream, F("AT+CIPCLOSE=%d\r\n"), connectionId);
    }
    loopstate = 0; // go back to waiting for +IPD
    return;
  }
  // prepare to send reply
  streamer.print('\0'); // null the end of the buffer so we can treat it as a string
  DIAG(F("%l WiFi(%d)->[%e] l(%d)\n\r"), millis(), connectionId, buffer, streamer.available() - 1);
  CommManager::send(wifiStream, F("AT+CIPSEND=%d,%d\r\n"), connectionId, streamer.available() - 1);
  loopTimeoutStart = millis();
  loopstate = 10; // non-blocking loop waits for > before sending
}