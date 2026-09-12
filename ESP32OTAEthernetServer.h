#ifndef ESP32OTAEthernetServer_h
#define ESP32OTAEthernetServer_h


#include <WiFi.h>
#include <SPI.h>
#include <Ethernet.h>

class ESP32EthernetServer : public EthernetServer {
public:
    ESP32EthernetServer(uint16_t port) : EthernetServer(port) {
        Serial.printf("XXX Port %d\n", port);
    }
    virtual void begin(uint16_t port = 0) override { EthernetServer::begin(); }
};


#endif