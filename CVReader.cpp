/*
 *  © 2020, Gregor Baues, Chris Harlow. All rights reserved.
 *  
 *  This is a basic, no frills CVreader example of a DCC++ compatible setup.
 *  There are more advanced examples in the examples folder i
 */
#include <Arduino.h>

// #include "DCCEX.h"
#include "MemoryFree.h"
#include "DIAG.h"

#include "NetworkInterface.h"

// DCCEXParser  serialParser;

/**
 * @brief User define callback for HTTP requests. The Network interface will provide for each http request a parsed request object 
 * and the client who send the request are provided. Its up to the user to use the req as he sees fits. Below is just a scaffold to
 * demonstrate the workings.
 * 
 * @param req     Parsed request object
 * @param client  Originator of the request to reply to 
 */


void httpRequestHandler(ParsedRequest *req, Client* client) {
  DIAG(F("\nParsed Request:"));
  DIAG(F("\nMethod:         [%s]"), req->method);
  DIAG(F("\nURI:            [%s]"), req->uri);
  DIAG(F("\nHTTP version:   [%s]"), req->version);
  DIAG(F("\nParameter count:[%d]\n"), *req->paramCount);
  
  // result = doSomething(); // obtain result to be send back; fully prepare the serialized HTTP response!

  // client->write(result);
}


void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. just in case
  }

  // DCC::begin(STANDARD_MOTOR_SHIELD);
  DIAG(F("\nFree RAM before network init: [%d]\n"),freeMemory());
  DIAG(F("\nNetwork Setup In Progress ...\n"));
  NetworkInterface::setup(WIFI, TCP, 8888);           // specify WIFI or ETHERNET depending on if you have Wifi or an EthernetShield; Wifi has to be on Serial1 UDP or TCP for the protocol
  NetworkInterface::setHttpCallback(httpRequestHandler);  // The network interface will provide and HTTP request object which can be used as well to send the reply. cf. example above
  
  // NetworkInterface::setup(WIFI, MQTT, 8888);     // sending over MQTT.
  // NetworkInterface::setup(WIFI, UDP, 8888);      // Setup without port will use the by default port 2560 :: DOES NOT WORK 
  // NetworkInterface::setup(WIFI);                 // setup without port and protocol will use by default TCP on port 2560 
  // NetworkInterface::setup();                     // all defaults ETHERNET, TCP on port 2560

  DIAG(F("\nNetwork Setup done ..."));
  
  
  DIAG(F("\nFree RAM after network init: [%d]\n"),freeMemory());
  DIAG(F("\nReady for DCC Commands ..."));
}

void loop()
{
  // DCC::loop();
  NetworkInterface::loop();

  // serialParser.loop(Serial);
}