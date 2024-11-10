/*
 *  © 2024 Harald Barth
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

#define BROADCASTTIME 15 //seconds

typedef enum _MDNSServiceProtocol_t 
{
  MDNSServiceTCP,
  MDNSServiceUDP
} MDNSServiceProtocol_t;

class MDNS {
public:
  MDNS(EthernetUDP& udp);
  ~MDNS();
  int begin(const IPAddress& ip,  char* name);
  int addServiceRecord(const char* name, uint16_t port, MDNSServiceProtocol_t proto);
  void run();
private:
  EthernetUDP *_udp;
  IPAddress _ipAddress;
  char* _name;
  char* _serviceName;
  char* _serviceProto;
  int _servicePort;
};
