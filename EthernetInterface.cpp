/*
 *  © 2020,Gregor Baues,  Chris Harlow. All rights reserved.
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

#include "config.h"
#include "defines.h" // This should be changed to DCCEX.h when possible
#if ETHERNET_ON == true
#include "EthernetInterface.h"
#include "DIAG.h"
#include "CommandDistributor.h"

EthernetInterface * EthernetInterface::singleton=NULL;
/**
 * @brief Setup Ethernet Connection
 * 
 */
void EthernetInterface::setup()
{
    singleton=new EthernetInterface();
    if (!singleton->connected) singleton=NULL; 
};


/**
 * @brief Aquire IP Address from DHCP and start server
 * 
 * @return true 
 * @return false 
 */
EthernetInterface::EthernetInterface()
{
    byte mac[]=MAC_ADDRESS;
    
    DIAG(F("\n+++++ Ethernet Setup "));
        connected=false;
   
    #ifdef IP_ADDRESS
    Ethernet.begin(mac, IP_ADDRESS);
    #else
    if (Ethernet.begin(mac) == 0)
    {
        DIAG(F("begin FAILED\n"));
        return;
    } 
    #endif
    DIAG(F("begin OK."));
     if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      DIAG(F("shield not found\n"));
      return;
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      DIAG(F("cable not connected\n"));
      return;
    }
    
    connected=true;
    
    IPAddress ip = Ethernet.localIP(); // reassign the obtained ip address

    server = new EthernetServer(LISTEN_PORT); // Ethernet Server listening on default port LISTEN_PORT
    server->begin();
  
    LCD(4,F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
    LCD(5,F("Port:%d"), LISTEN_PORT);

    outboundRing=new RingStream(OUTBOUND_RING_SIZE);     
}

/**
 * @brief Main loop for the EthernetInterface
 * 
 */
void EthernetInterface::loop()
{
    if (!singleton) return;
    
    switch (Ethernet.maintain())
    {
    case 1:
        //renewed fail
        DIAG(F("\nEthernet Error: renewed fail\n"));
        singleton=NULL;
        return;

    case 3:
        //rebind fail
        DIAG(F("Ethernet Error: rebind fail\n"));
        singleton=NULL;
        return;

    default:
        //nothing happened
        break;
    }

   singleton->loop2();

}

 void EthernetInterface::loop2()
{
    // get client from the server
    EthernetClient client = server->accept();

    // check for new client
    if (client)
    {
        if (Diag::ETHERNET) DIAG(F("\nEthernet: New client "));
        byte socket;
        for (socket = 0; socket < MAX_SOCK_NUM; socket++)
        {
            if (!clients[socket])
            {
                // On accept() the EthernetServer doesn't track the client anymore
                // so we store it in our client array
                if (Diag::ETHERNET) DIAG(F("%d\n"),socket);
                clients[socket] = client;
                break;
            }
        }
        if (socket==MAX_SOCK_NUM) DIAG(F("new Ethernet OVERFLOW\n")); 
    }

    // check for incoming data from all possible clients
    for (byte socket = 0; socket < MAX_SOCK_NUM; socket++)
    {
        if (clients[socket]) {
        
        int available=clients[socket].available();
        if (available > 0) {
            if (Diag::ETHERNET)  DIAG(F("\nEthernet: available socket=%d,avail=%d,count="), socket, available);
            // read bytes from a client
            int count = clients[socket].read(buffer, MAX_ETH_BUFFER);
            buffer[count] = '\0'; // terminate the string properly
            if (Diag::ETHERNET) DIAG(F("%d:%e\n"), socket,buffer);
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
      if (Diag::ETHERNET)  DIAG(F("\nEthernet: disconnect %d \n"), socket);             
     }
    }
    
    // handle at most 1 outbound transmission 
    int socketOut=outboundRing->read();
    if (socketOut>=0) {
      int count=outboundRing->count();
      if (Diag::ETHERNET) DIAG(F("Ethernet reply socket=%d, count=:%d\n"), socketOut,count);
      for(;count>0;count--)  clients[socketOut].write(outboundRing->read());
      clients[socketOut].flush(); //maybe 
    }
}
#endif
