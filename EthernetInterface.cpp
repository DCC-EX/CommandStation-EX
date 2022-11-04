/*
 *  © 2022 Bruno Sanches
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  © 2020 Gregor Baues
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
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
 * 
 */
#include "defines.h" 
#if ETHERNET_ON == true
#include "EthernetInterface.h"
#include "DIAG.h"
#include "CommandDistributor.h"
#include "DCCTimer.h"

EthernetInterface * EthernetInterface::singleton=NULL;
/**
 * @brief Setup Ethernet Connection
 * 
 */
void EthernetInterface::setup()
{
  if (singleton!=NULL) {
    DIAG(F("Prog Error!"));
    return;
  }
  if ((singleton=new EthernetInterface()))
    return;
  DIAG(F("Ethernet not initialized"));
};


/**
 * @brief Aquire IP Address from DHCP and start server
 * 
 * @return true 
 * @return false 
 */
EthernetInterface::EthernetInterface()
{
    byte mac[6];
    DCCTimer::getSimulatedMacAddress(mac);
    connected=false;
   
    #ifdef IP_ADDRESS
    Ethernet.begin(mac, IP_ADDRESS);
    #else
    if (Ethernet.begin(mac) == 0)
    {
        DIAG(F("Ethernet.begin FAILED"));
        return;
    } 
    #endif       
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
      DIAG(F("Ethernet shield not detected or is a W5100"));

    unsigned long startmilli = millis();
    while ((millis() - startmilli) < 5500) { // Loop to give time to check for cable connection
      if (Ethernet.linkStatus() == LinkON)
	break;
      DIAG(F("Ethernet waiting for link (1sec) "));
      delay(1000);
    }
}

/**
 * @brief Cleanup any resources
 * 
 * @return none
 */
EthernetInterface::~EthernetInterface() {
  delete server;
  delete outboundRing;
}

/**
 * @brief Main loop for the EthernetInterface
 * 
 */
void EthernetInterface::loop() {
  if(!singleton || (!singleton->checkLink()))
    return;

  switch (Ethernet.maintain()) {
  case 1:
    //renewed fail
    DIAG(F("Ethernet Error: renewed fail"));
    singleton=NULL;
    return;
  case 3:
    //rebind fail
    DIAG(F("Ethernet Error: rebind fail"));
    singleton=NULL;
    return;
  default:
    //nothing happened
    break;
  }
  singleton->loop2();
}

/**
 * @brief Checks ethernet link cable status and detects when it connects / disconnects
 * 
 * @return true when cable is connected, false otherwise
 */
bool EthernetInterface::checkLink() {
  if (Ethernet.linkStatus() != LinkOFF) { // check for not linkOFF instead of linkON as the W5100 does return LinkUnknown
    //if we are not connected yet, setup a new server
    if(!connected) {
      DIAG(F("Ethernet cable connected"));
      connected=true;
      IPAddress ip = Ethernet.localIP(); // reassign the obtained ip address
      server = new EthernetServer(IP_PORT); // Ethernet Server listening on default port IP_PORT
      server->begin();
      LCD(4,F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
      LCD(5,F("Port:%d"), IP_PORT);
      // only create a outboundRing it none exists, this may happen if the cable
      // gets disconnected and connected again
      if(!outboundRing)
	outboundRing=new RingStream(OUTBOUND_RING_SIZE);
    }
    return true;
  } else { // connected
    DIAG(F("Ethernet cable disconnected"));
    connected=false;
    //clean up any client
    for (byte socket = 0; socket < MAX_SOCK_NUM; socket++) {
      if(clients[socket].connected())
	clients[socket].stop();
    }
    // tear down server
    delete server;
    server = nullptr;
    LCD(4,F("IP: None"));
  }
  return false;
}

void EthernetInterface::loop2() {
    // get client from the server
    EthernetClient client = server->accept();

    // check for new client
    if (client)
    {
        if (Diag::ETHERNET) DIAG(F("Ethernet: New client "));
        byte socket;
        for (socket = 0; socket < MAX_SOCK_NUM; socket++)
        {
            if (!clients[socket])
            {
                // On accept() the EthernetServer doesn't track the client anymore
                // so we store it in our client array
                if (Diag::ETHERNET) DIAG(F("Socket %d"),socket);
                clients[socket] = client;
                break;
            }
        }
        if (socket==MAX_SOCK_NUM) DIAG(F("new Ethernet OVERFLOW")); 
    }

    // check for incoming data from all possible clients
    for (byte socket = 0; socket < MAX_SOCK_NUM; socket++)
    {
        if (clients[socket]) {
        
        int available=clients[socket].available();
        if (available > 0) {
            if (Diag::ETHERNET)  DIAG(F("Ethernet: available socket=%d,avail=%d"), socket, available);
            // read bytes from a client
            int count = clients[socket].read(buffer, MAX_ETH_BUFFER);
            buffer[count] = '\0'; // terminate the string properly
            if (Diag::ETHERNET) DIAG(F(",count=%d:%e"), socket,buffer);
            // execute with data going directly back
            outboundRing->mark(socket); 
            CommandDistributor::parse(socket,buffer,outboundRing);
            outboundRing->commit();
            return; // limit the amount of processing that takes place within 1 loop() cycle. 
          }
        }
    }

    // stop any clients which disconnect
   for (int socket = 0; socket<MAX_SOCK_NUM; socket++) {
     if (clients[socket] && !clients[socket].connected()) {
      clients[socket].stop();
      CommandDistributor::forget(socket);          
      if (Diag::ETHERNET)  DIAG(F("Ethernet: disconnect %d "), socket);             
     }
    }
    
    // handle at most 1 outbound transmission 
    int socketOut=outboundRing->read();
    if (socketOut>=0) {
      int count=outboundRing->count();
      if (Diag::ETHERNET) DIAG(F("Ethernet reply socket=%d, count=:%d"), socketOut,count);
      for(;count>0;count--)  clients[socketOut].write(outboundRing->read());
      clients[socketOut].flush(); //maybe 
    }
}
#endif
