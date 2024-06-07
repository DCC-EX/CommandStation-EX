 /*
 *  © 2022 Paul M. Antoine
 *  © 2021 Chris Harlow
 *  © 2022 Harald Barth
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
  inCommandPayload=false; 
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
  Serial2.begin(115200);
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
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // GPIO 16 RXD2; GPIO 17 TXD2 on ESP32
#endif
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
    while (serial->available()) {
        char ch = serial->read();
        if (ch == '<') {
            inCommandPayload = true;
            bufferLength = 0;
            buffer[0] = '\0';
        }
        else if  (inCommandPayload) {
	  if (bufferLength <  (COMMAND_BUFFER_SIZE-1))
	    buffer[bufferLength++] = ch;
	  if (ch == '>') {
	    buffer[bufferLength] = '\0';
	    DCCEXParser::parse(serial, buffer, NULL); 
	    inCommandPayload = false;
	    break;
	  }
        }
    }
    
}
