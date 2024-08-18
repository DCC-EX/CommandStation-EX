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
#include "WiThrottle.h"
#include "DCCTimer.h"

extern void looptimer(unsigned long timeout, const FSH* message);

bool EthernetInterface::connected=false;
EthernetServer * EthernetInterface::server= nullptr;
EthernetClient EthernetInterface::clients[MAX_SOCK_NUM];                // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on the Shield
uint8_t EthernetInterface::buffer[MAX_ETH_BUFFER+1];                    // buffer used by TCP for the recv
RingStream * EthernetInterface::outboundRing = nullptr;

/**
 * @brief Setup Ethernet Connection
 * 
 */
void EthernetInterface::setup()
{
    connected=false;

    byte mac[6];
    DCCTimer::getSimulatedMacAddress(mac);
    DIAG(F("Ethernet begin"));
    if (Ethernet.begin(mac) == 0)
    {
        DIAG(F("Ethernet.begin FAILED"));
        return;
    } 
    auto ip = Ethernet.localIP();    // look what IP was obtained (dynamic or static)
    server = new EthernetServer(IP_PORT); // Ethernet Server listening on default port IP_PORT
    server->begin();
    LCD(4,F("IP: %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
    LCD(5,F("Port:%d"), IP_PORT);
    outboundRing=new RingStream(OUTBOUND_RING_SIZE);
    connected=true;
}


/**
 * @brief Main loop for the EthernetInterface
 * 
 */
void EthernetInterface::loop()
{
    if (!connected) return;
    //DIAG(F("maintain"));
    switch (Ethernet.maintain()) {
    case 1:
        //renewed fail
        DIAG(F("Ethernet Error: renewed fail"));
        connected=false;
        return;
    case 3:
        //rebind fail
        DIAG(F("Ethernet Error: rebind fail"));
        connected=false;
        return;
    default:
        //nothing happened
        //DIAG(F("maintained"));
        break;
    }
    // looptimer(8000, F("Ethloop after maintain"));

    // get client from the server
    auto client = server->accept();

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
      if (!clients[socket]) continue; // socket is not in use
	
	    // read any bytes from this client
	    auto count = clients[socket].read(buffer, MAX_ETH_BUFFER);
      
      if (count<0) continue;  // -1 indicates nothing to read
	    
      if (count > 0) {  // we have incoming data 
	      buffer[count] = '\0'; // terminate the string properly
	      if (Diag::ETHERNET) DIAG(F("Ethernet s=%d, c=%d b=:%e"), socket, count, buffer);
	      // execute with data going directly back
	      CommandDistributor::parse(socket,buffer,outboundRing);
	      //looptimer(5000, F("Ethloop2 parse"));
	      return; // limit the amount of processing that takes place within 1 loop() cycle. 
	    }
	    
      // count=0 The client has disconnected
	    clients[socket].stop();
	    CommandDistributor::forget(socket);
	    if (Diag::ETHERNET)  DIAG(F("Ethernet: disconnect %d "), socket);
	  }
	
    //looptimer(8000, F("Ethloop2 after incoming"));

    WiThrottle::loop(outboundRing);
    //looptimer(8000, F("Ethloop after Withrottleloop"));

    // handle at most 1 outbound transmission 
    auto socketOut=outboundRing->read();
    if (socketOut<0) return;  // no outbound pending

    if (socketOut >= MAX_SOCK_NUM) {
      // This is a catastrophic code failure and unrecoverable.  
      DIAG(F("Ethernet outboundRing s=%d error"), socketOut);
      connected=false;
      return;
    } 

    auto count=outboundRing->count();
    {
	    char tmpbuf[count+1]; // one extra for '\0'
	    for(int i=0;i<count;i++) {
	      tmpbuf[i] = outboundRing->read();
	    }
	    tmpbuf[count]=0;
	    if (Diag::ETHERNET) DIAG(F("Ethernet reply s=%d, c=%d, b:%e"),
                              socketOut,count,tmpbuf);
	    clients[socketOut].write(tmpbuf,count);
    }
    
    //looptimer(8000, F("Ethloop after outbound"));
}
#endif
