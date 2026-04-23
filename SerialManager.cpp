 /*
 *  © 2022 Paul M. Antoine
 *  © 2021 Chris Harlow
 *  © 2022 2024 Harald Barth
 *  All rights reserved.
 *  
 *  This file is part of DCC++EX
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

#include "SerialManager.h"
#include "DCCEXParser.h"
#include "StringFormatter.h"
#include "WiThrottle.h"
#include "DIAG.h"

#ifdef ARDUINO_ARCH_ESP32
#ifdef SERIAL_BT_COMMANDS
#include <BluetoothSerial.h>
//#include <BleSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error No Bluetooth library available
#endif //ENABLED
BluetoothSerial SerialBT;
//BleSerial SerialBT;
#endif //COMMANDS
#endif //ESP32


SerialManager * SerialManager::first=NULL;

SerialManager::SerialManager(Stream * myserial) {
  serial=myserial;
  next=first;
  first=this;
  bufferLength=0;
  inCommandPayload=PAYLOAD_WAITING; 
  withrottleInUse=false;
} 

void SerialManager::init() {
  USB_SERIAL.begin(115200);
  while (!USB_SERIAL && millis() < 5000); // wait max 5s for Serial to start
  new SerialManager(&USB_SERIAL);
  
#ifdef SERIAL6_COMMANDS
  Serial6.begin(115200);
  new SerialManager(&Serial6);
#endif
#ifdef SERIAL5_COMMANDS
  Serial5.begin(115200);
  new SerialManager(&Serial5);
#endif
#ifdef SERIAL4_COMMANDS
  Serial4.begin(115200);
  new SerialManager(&Serial4);
#endif
#ifdef SERIAL3_COMMANDS
  Serial3.begin(115200);
  new SerialManager(&Serial3);
#endif
#ifdef SERIAL2_COMMANDS
#ifdef ARDUINO_ARCH_ESP32
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // GPIO 16 RXD2; GPIO 17 TXD2 on ESP32
#else  // not ESP32
  Serial2.begin(115200);
#endif // ESP32
  new SerialManager(&Serial2);
#endif
#ifdef SERIAL1_COMMANDS
  Serial1.begin(115200);
  new SerialManager(&Serial1);
#endif
#ifdef SERIAL_BT_COMMANDS
  {
    //SerialBT.setPin("6666"); // choose other pin
    uint64_t chipid = ESP.getEfuseMac();
    char idstr[16] = {0};
    snprintf(idstr, 15, "DCCEX-%08X",
	     __builtin_bswap32((uint32_t)(chipid>>16)));
    SerialBT.begin(idstr);
    new SerialManager(&SerialBT);
    delay(1000);
  }
#endif
#ifdef SABERTOOTH
#ifdef ARDUINO_ARCH_ESP32
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // GPIO 16 RXD2; GPIO 17 TXD2 on ESP32
#else
  Serial2.begin(9600);
#endif
#endif
}

void SerialManager::broadcastWithrottle(char * stringBuffer) {
    // send a withrottle broadcast over whichever serial is handling the conduit with a withrottle client, which is identified by the withrottleInUse flag.  We use a special format to allow the conduit parser to identify these and avoid loops.  The format is [*]message~ where [*] identifies this as a withrottle broadcast and ~ identifies the end of the message.
    for (SerialManager * s=first;s;s=s->next) {
      if (s->withrottleInUse) StringFormatter::send(s->serial,F("[*]%s~"),stringBuffer);
    }
}

void SerialManager::broadcast(char * stringBuffer) {
    for (SerialManager * s=first;s;s=s->next) s->broadcast2(stringBuffer);
}
void SerialManager::broadcast2(char * stringBuffer) {
    serial->print(stringBuffer);
}

void SerialManager::loop() {
    for (SerialManager * s=first;s;s=s->next) s->loop2();
}

void SerialManager::loop2() {
  if (withrottleInUse && serial->available()==0) {
    // if this serial is doing nothing and but withrottle is in use it needs a loop() call to check for changes
    WiThrottle::loop(serial);
    return;
  }

  while (serial->available()) {
    char ch = serial->read();
    bool overflow = bufferLength >= COMMAND_BUFFER_SIZE;

    switch (inCommandPayload) {
      case PAYLOAD_WAITING:
        if (ch == '<') inCommandPayload = PAYLOAD_DCCEX; // start of DCC++EX command
        else if (ch == '[') inCommandPayload = PAYLOAD_TAGGED; // start of tagged payload, used for withrottle conduit messages
        else break; // ignore for now
          
        bufferLength=0; // reset buffer for new command
        buffer[bufferLength++]=ch;
        break;

      case PAYLOAD_DCCEX:
        if (!overflow) buffer[bufferLength++] = ch;          // advance bufferLength
        if(ch=='"') {
          inCommandPayload=PAYLOAD_STRING; // start of string, which can contain > so we need to wait for closing " before we can parse
          break;
        }
        if (ch == '>') { // end of DCC++EX command
         buffer[bufferLength] = '\0';
         DCCEXParser::parse(serial, buffer, NULL);
         inCommandPayload = PAYLOAD_WAITING;
       }
       break;
       
    case PAYLOAD_STRING:
       if (!overflow) buffer[bufferLength++] = ch;          // advance bufferLength
       if (ch == '"') inCommandPayload = PAYLOAD_DCCEX; // end of string, back to normal parsing
       break;
   
    case PAYLOAD_TAGGED:
        if (ch=='~') { // end of tagged input
          buffer[bufferLength]='\0';
          withrottleInUse=true; // assume any tagged input is from withrottle until we have reason to think otherwise
          WiThrottle::parseConduit(serial, buffer);  // buffer parsed with trailing '>'
          inCommandPayload = PAYLOAD_WAITING;
          break;
        }
        if (!overflow) buffer[bufferLength++] = ch;          // advance bufferLength
        break;
    }
  }
}
