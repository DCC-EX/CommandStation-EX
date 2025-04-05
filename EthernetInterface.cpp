/*
 *  © 2024 Morten "Doc" Nielsen
 *  © 2023-2024 Paul M. Antoine
 *  © 2022 Bruno Sanches
 *  © 2021 Fred Decker
 *  © 2020-2022 Harald Barth
 *  © 2020-2024 Chris Harlow
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
#if __has_include("MDNS_Generic.h")
#include "MDNS_Generic.h"
#define DO_MDNS
EthernetUDP udp;
MDNS mdns(udp);
#endif

// extern void looptimer(unsigned long timeout, const FSH* message);
#define looptimer(a, b)

bool EthernetInterface::connected = false;
EthernetServer* EthernetInterface::server = nullptr;
EthernetClient EthernetInterface::clients[MAX_SOCK_NUM];  // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the
                                                          // chipset used on the Shield
bool EthernetInterface::inUse[MAX_SOCK_NUM];  // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on
                                              // the Shield
uint8_t EthernetInterface::buffer[MAX_ETH_BUFFER + 1];  // buffer used by TCP for the recv
RingStream* EthernetInterface::outboundRing = nullptr;

/**
 * @brief Setup Ethernet Connection
 *
 */

void EthernetInterface::setup() {
  DIAG(
      F("Ethernet starting"
#ifdef DO_MDNS
        " (with mDNS)"
#endif
        " Please be patient, especially if no cable is connected!"));

#ifdef STM32_ETHERNET
  // Set a HOSTNAME for the DHCP request - a nice to have, but hard it seems on LWIP for STM32
  // The default is "lwip", which is **always** set in STM32Ethernet/src/utility/ethernetif.cpp
  // for some reason. One can edit it to instead read:
  //      #if LWIP_NETIF_HOSTNAME
  //      /* Initialize interface hostname */
  //      if (netif->hostname == NULL)
  //         netif->hostname = "lwip";
  //      #endif /* LWIP_NETIF_HOSTNAME */
  // Which seems more useful! We should propose the patch... so the following line actually works!
  netif_set_hostname(&gnetif, WIFI_HOSTNAME);  // Should probably be passed in the contructor...
#endif

  byte mac[6];
  DCCTimer::getSimulatedMacAddress(mac);

#ifdef IP_ADDRESS
  static IPAddress myIP(IP_ADDRESS);
  Ethernet.begin(mac, myIP);
#else
  if (Ethernet.begin(mac) == 0) {
    LCD(4, F("IP: No DHCP"));
    return;
  }
#endif

  auto ip = Ethernet.localIP();  // look what IP was obtained (dynamic or static)
  if (!ip) {
    LCD(4, F("IP: None"));
    return;
  }
  server = new EthernetServer(IP_PORT);  // Ethernet Server listening on default port IP_PORT
  server->begin();

// Arrange display of IP address and port
#ifdef LCD_DRIVER
  const byte lcdData[] = {LCD_DRIVER};
  const bool wideDisplay = lcdData[1] >= 24;  // data[1] is cols.
#else
  const bool wideDisplay = true;
#endif
  if (wideDisplay) {
    // OLEDS or just usb diag is ok on one line.
    LCD(4, F("IP %d.%d.%d.%d:%d"), ip[0], ip[1], ip[2], ip[3], IP_PORT);
  } else {  // LCDs generally too narrow, so take 2 lines
    LCD(4, F("IP %d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
    LCD(5, F("Port %d"), IP_PORT);
  }

  outboundRing = new RingStream(OUTBOUND_RING_SIZE);
#ifdef DO_MDNS
  mdns.begin(Ethernet.localIP(), WIFI_HOSTNAME);  // hostname
  mdns.addServiceRecord(WIFI_HOSTNAME "._withrottle", IP_PORT, MDNSServiceTCP);
  // Not sure if we need to run it once, but just in case!
  mdns.run();
#endif
  connected = true;
}

#if defined(STM32_ETHERNET)
void EthernetInterface::acceptClient() {  // STM32 version
  auto client = server->available();
  if (!client)
    return;
  // check for existing client
  for (byte socket = 0; socket < MAX_SOCK_NUM; socket++)
    if (inUse[socket] && client == clients[socket])
      return;

  // new client
  for (byte socket = 0; socket < MAX_SOCK_NUM; socket++) {
    if (!inUse[socket]) {
      clients[socket] = client;
      inUse[socket] = true;
      if (Diag::ETHERNET)
        DIAG(F("Ethernet: New client socket %d"), socket);
      return;
    }
  }
  DIAG(F("Ethernet OVERFLOW"));
}
#else
void EthernetInterface::acceptClient() {  // non-STM32 version
  auto client = server->accept();
  if (!client)
    return;
  auto socket = client.getSocketNumber();
  clients[socket] = client;
  inUse[socket] = true;
  if (Diag::ETHERNET)
    DIAG(F("Ethernet: New client socket %d"), socket);
}
#endif

void EthernetInterface::dropClient(byte socket) {
  clients[socket].stop();
  inUse[socket] = false;
  CommandDistributor::forget(socket);
  if (Diag::ETHERNET)
    DIAG(F("Ethernet: Disconnect %d "), socket);
}

/**
 * @brief Main loop for the EthernetInterface
 *
 */
void EthernetInterface::loop() {
  if (!connected)
    return;
  looptimer(5000, F("E.loop"));

  static bool warnedAboutLink = false;
  if (Ethernet.linkStatus() == LinkOFF) {
    if (warnedAboutLink)
      return;
    DIAG(F("Ethernet link OFF"));
    warnedAboutLink = true;
    return;
  }
  looptimer(5000, F("E.loop warn"));

  // link status must be ok here
  if (warnedAboutLink) {
    DIAG(F("Ethernet link RESTORED"));
    warnedAboutLink = false;
  }

#ifdef DO_MDNS
  // Always do this because we don't want traffic to intefere with being found!
  mdns.run();
  looptimer(5000, F("E.mdns"));

#endif

  //
  switch (Ethernet.maintain()) {
    case 1:
      // renewed fail
      DIAG(F("Ethernet Error: renewed fail"));
      connected = false;
      return;
    case 3:
      // rebind fail
      DIAG(F("Ethernet Error: rebind fail"));
      connected = false;
      return;
    default:
      // nothing happened
      // DIAG(F("maintained"));
      break;
  }
  looptimer(5000, F("E.maintain"));

  // get client from the server
  acceptClient();

  // handle disconnected sockets because STM32 library doesnt
  // do the read==0 response.
  for (byte socket = 0; socket < MAX_SOCK_NUM; socket++) {
    if (inUse[socket] && !clients[socket].connected())
      dropClient(socket);
  }

  // check for incoming data from all possible clients
  for (byte socket = 0; socket < MAX_SOCK_NUM; socket++) {
    if (!inUse[socket])
      continue;  // socket is not in use

    // read any bytes from this client
    auto count = clients[socket].read(buffer, MAX_ETH_BUFFER);

    if (count < 0)
      continue;  // -1 indicates nothing to read

    if (count > 0) {         // we have incoming data
      buffer[count] = '\0';  // terminate the string properly
      if (Diag::ETHERNET)
        DIAG(F("Ethernet s=%d, c=%d b=:%e"), socket, count, buffer);
      // execute with data going directly back
      CommandDistributor::parse(socket, buffer, outboundRing);
      // looptimer(5000, F("Ethloop2 parse"));
      return;  // limit the amount of processing that takes place within 1 loop() cycle.
    }

    // count=0 The client has disconnected
    dropClient(socket);
  }

  WiThrottle::loop(outboundRing);

  // handle at most 1 outbound transmission
  auto socketOut = outboundRing->read();
  if (socketOut < 0)
    return;  // no outbound pending

  if (socketOut >= MAX_SOCK_NUM) {
    // This is a catastrophic code failure and unrecoverable.
    DIAG(F("Ethernet outboundRing s=%d error"), socketOut);
    connected = false;
    return;
  }

  auto count = outboundRing->count();
  {
    char tmpbuf[count + 1];  // one extra for '\0'
    for (int i = 0; i < count; i++) {
      tmpbuf[i] = outboundRing->read();
    }
    tmpbuf[count] = 0;
    if (inUse[socketOut]) {
      if (Diag::ETHERNET)
        DIAG(F("Ethernet reply s=%d, c=%d, b:%e"), socketOut, count, tmpbuf);
      clients[socketOut].write(tmpbuf, count);
    }
  }
}
#endif
