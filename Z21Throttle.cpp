/*
 *  © 2023 Thierry Paris / Locoduino
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX-Labox
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
#include <Arduino.h>
#include "defines.h"
#include <WiFi.h>
#include "Z21Throttle.h"
#include "DCC.h"
#include "WifiESP32.h"
#include "DCCWaveform.h"
#include "StringFormatter.h"
#include "Turnouts.h"
#include "DIAG.h"
#include "GITHUB_SHA.h"
#include "version.h"
#include "EXRAIL2.h"
#include "CommandDistributor.h"
#include "TrackManager.h"
#include "DCCTimer.h"
#ifdef USE_HMI
	#include "hmi.h"
#endif

static std::vector<NetworkClientUDP> clientsUDP; // a list to hold all UDP clients

Z21Throttle *Z21Throttle::firstThrottle=NULL;
byte Z21Throttle::commBuffer[100];
byte Z21Throttle::replyBuffer[20];

Z21Throttle* Z21Throttle::readWriteThrottle = NULL;
int Z21Throttle::cvAddress = -1;
int Z21Throttle::cvValue = -1;

void printClientsUDP();

WiFiUDP NetworkClientUDP::client;

#define LOOPLOCOS(THROTTLECHAR, CAB)  for (int loco=0;loco<MAX_MY_LOCO;loco++) \
      if ((myLocos[loco].throttle==THROTTLECHAR || '*'==THROTTLECHAR) && (CAB<0 || myLocos[loco].cab==CAB))

void Z21Throttle::setup(IPAddress ip, int port) {
	uint8_t ret = NetworkClientUDP::client.begin(ip, port);
    if(ret == 1) DIAG(F("UDP Connection started port %d. Z21 apps are available."), port);
    else DIAG(F("UDP Connection failed. Z21 apps not available."));
	NetworkClientUDP::client.flush();
}

int readUdpPacket() {
	byte udp[UDPBYTE_SIZE];
	memset(udp, 0, UDPBYTE_SIZE);
	int len = NetworkClientUDP::client.read(udp, UDPBYTE_SIZE);
	if (len > 0) {
		for (int clientId = 0; clientId < clientsUDP.size(); clientId++) {
		if (clientsUDP[clientId].inUse)
			if (clientsUDP[clientId].remoteIP == NetworkClientUDP::client.remoteIP() && clientsUDP[clientId].remotePort == NetworkClientUDP::client.remotePort()) {
				clientsUDP[clientId].pudpBuffer->PushBytes(udp, len);
			return clientId;
			}
		}
	}

	return -1;
}

void Z21Throttle::loop() {
	int clientId = 0;

	// loop over all clients and remove inactive
	for (clientId = 0; clientId < clientsUDP.size(); clientId++) {
		// check if client is there and alive
		if (clientsUDP[clientId].inUse && !clientsUDP[clientId].connected) {
			if (Diag::Z21THROTTLE) DIAG(F("Remove UDP client %d"), clientId);
			clientsUDP[clientId].inUse = false;
			printClientsUDP();
		}
	}

	int len = NetworkClientUDP::client.parsePacket();
	if (len > 0) {
		int clientId = 0;
		for (; clientId < clientsUDP.size(); clientId++) {
			if (clientsUDP[clientId].inUse) {
				if (clientsUDP[clientId].remoteIP == NetworkClientUDP::client.remoteIP() && clientsUDP[clientId].remotePort == NetworkClientUDP::client.remotePort()) {
					//if (Diag::Z21THROTTLEVERBOSE) DIAG(F("UDP client %d : %s Already connected"), clientId, clientsUDP[clientId].remoteIP.toString().c_str());
					break;
				}
			}
		}

		if (clientId >= clientsUDP.size()) {
			NetworkClientUDP nc;
			nc.remoteIP = NetworkClientUDP::client.remoteIP();
			nc.remotePort = NetworkClientUDP::client.remotePort();
			nc.connected = true;
			nc.inUse = true;

			clientsUDP.push_back(nc);
			if (Diag::Z21THROTTLE) DIAG(F("New UDP client %d, %s"), clientId, nc.remoteIP.toString().c_str());
			printClientsUDP();
			#ifdef USE_HMI
			if (hmi::CurrentInterface != NULL) hmi::CurrentInterface->NewClient(clientId, nc.remoteIP, 0);
			#endif
			// Fleischmann/Roco Android app starts with Power on !
			TrackManager::setMainPower(POWERMODE::ON);
		}

		clientId = readUdpPacket();

		if (clientId >= 0) {
			if (clientsUDP[clientId].ok()) {        
				Z21Throttle* pThrottle = getOrAddThrottle(clientId); 

				if (pThrottle != NULL)
					pThrottle->parse();
			}
		}
	}
}

/** Print the list of assigned locomotives. */
void Z21Throttle::printLocomotives(bool addTab) {
	if (!Diag::Z21THROTTLE)
		return;

	DIAG(F("      Locomotives ------------------"));
	for (int loco = 0; loco < MAX_MY_LOCO; loco++)
		if (myLocos[loco].throttle != '\0')
			DIAG(F("%s         %d : cab %d on throttle %c"), addTab ? "   ":"", loco, myLocos[loco].cab, myLocos[loco].throttle);
}

/** Print the list of assigned locomotives. */
void printClientsUDP() {
	if (!Diag::Z21THROTTLE) return;

	DIAG(F("      UDP Clients ------------------"));
	for (int clientId = 0; clientId < clientsUDP.size(); clientId++)
		if (clientsUDP[clientId].ok())
			DIAG(F("         %d %s: %s:%d"), clientId, clientsUDP[clientId].connected?"Connected":"Not connected", clientsUDP[clientId].remoteIP.toString().c_str(), clientsUDP[clientId].remotePort);
		else
			DIAG(F("         %d unused"), clientId);
}

/** Print the list of assigned locomotives. */
void Z21Throttle::printThrottles(bool inPrintLocomotives) {
	if (!Diag::Z21THROTTLE)	return;

	DIAG(F("      Z21 Throttles ---------------"));
	for (Z21Throttle* wt = firstThrottle; wt != NULL; wt = wt->nextThrottle) {
		if (wt->clientid == -1)
			DIAG(F("         unused"));
		else {
			DIAG(F("         %d : %d.%d.%d.%d:%d"), wt->clientid, 
			clientsUDP[wt->clientid].remoteIP[0], 
			clientsUDP[wt->clientid].remoteIP[1], 
			clientsUDP[wt->clientid].remoteIP[2], 
			clientsUDP[wt->clientid].remoteIP[3], 
			clientsUDP[wt->clientid].remotePort);

			if (inPrintLocomotives)
				wt->printLocomotives(true);
		}
	}
}

Z21Throttle* Z21Throttle::getOrAddThrottle(int clientId) {
	for (Z21Throttle* wt = firstThrottle; wt != NULL ; wt = wt->nextThrottle)  {
		if (wt->clientid == clientId)
			return wt; 
	}

	Z21Throttle *p = new Z21Throttle(clientId);
	printThrottles(false);
	return p;
}

void Z21Throttle::forget( byte clientId) {
	for (Z21Throttle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
    	if (wt->clientid==clientId) {
			delete wt;
			break; 
		}
}

bool Z21Throttle::isThrottleInUse(int cab) {
	for (Z21Throttle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle)  
		if (wt->areYouUsingThrottle(cab)) return true;

	return false;
}

bool Z21Throttle::areYouUsingThrottle(int cab) {
	LOOPLOCOS('*', cab) { // see if I have this cab in use
		return true;
	}

	return false;
}

// One instance of Z21Throttle per connected client, so we know what the locos are 
 
Z21Throttle::Z21Throttle(int inClientId) {
	if (Diag::Z21THROTTLE) DIAG(F("New Z21Throttle for client UDP %d"), clientid); 
	nextThrottle=firstThrottle;
	firstThrottle= this;
	clientid = inClientId;
	initSent=false; // prevent sending heartbeats before connection completed
	turnoutListHash = -1;  // make sure turnout list is sent once
	exRailSent=false;
	mostRecentCab=0;     
	for (int loco=0;loco<MAX_MY_LOCO; loco++)
		myLocos[loco].throttle='\0';
}

Z21Throttle::~Z21Throttle() {
	if (Diag::Z21THROTTLE) DIAG(F("Deleting Z21Throttle client UDP %d"),this->clientid);
	if (firstThrottle== this) {
		firstThrottle=this->nextThrottle;
		return;
	}

	for (Z21Throttle* wt=firstThrottle; wt!=NULL ; wt=wt->nextThrottle) {
		if (wt->nextThrottle==this) {
			wt->nextThrottle=this->nextThrottle;
			return;  
		}
	}
}

void Z21Throttle::write(byte* inpData, int inLengthData) {
	size_t size = 0;

//	if (this->dontReply)
//		return;
	
	NetworkClientUDP::client.beginPacket(clientsUDP[this->clientid].remoteIP, clientsUDP[this->clientid].remotePort);
	size = NetworkClientUDP::client.write(inpData, inLengthData);
	NetworkClientUDP::client.endPacket();

	if (Diag::Z21THROTTLEDATA) DIAG(F("Z21 Throttle %d : %s SENT 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x"), clientid,
                          size == 0 ? "BINARY NOT" :"",
                          (inLengthData > 0)?inpData[0]:0,
                          (inLengthData > 1)?inpData[1]:0,
                          (inLengthData > 2)?inpData[2]:0,
                          (inLengthData > 3)?inpData[3]:0,
                          (inLengthData > 4)?inpData[4]:0,
                          (inLengthData > 5)?inpData[5]:0,
                          (inLengthData > 6)?inpData[6]:0,
                          (inLengthData > 7)?inpData[7]:0,
                          (inLengthData > 8)?inpData[8]:0,
                          (inLengthData > 9)?inpData[9]:0);
}

// sizes : [       2        ][       2        ][inLengthData]
// bytes : [length1, length2][Header1, Header2][Data........]
bool Z21Throttle::notify(unsigned int inHeader, byte* inpData, unsigned int inLengthData, bool inXorInData) {
	int realLength = (inLengthData + 4 + (inXorInData == false ? 1 : 0));

	Z21Throttle::commBuffer[0] = realLength % 256;
	Z21Throttle::commBuffer[1] = realLength / 256;
	Z21Throttle::commBuffer[2] = inHeader % 256;
	Z21Throttle::commBuffer[3] = inHeader / 256;
	memcpy(Z21Throttle::commBuffer + 4, inpData, inLengthData);

	if (!inXorInData) {    // if xor byte not included in data, compute and write it !
		byte xxor = 0;

		for (unsigned int i = 0; i < inLengthData; i++)
			xxor ^= inpData[i];

		Z21Throttle::commBuffer[inLengthData+4] = xxor;
	}

	write(Z21Throttle::commBuffer, realLength);

	return true;
}

// sizes : [       2        ][       2        ][   1   ][inLengthData]
// bytes : [length1, length2][Header1, Header2][XHeader][Data........]
bool Z21Throttle::notify(unsigned int inHeader, unsigned int inXHeader, byte* inpData, unsigned int inLengthData, bool inXorInData) {
	int realLength = (inLengthData + 5 + (inXorInData == false ? 1 : 0));

	Z21Throttle::commBuffer[0] = realLength % 256;
	Z21Throttle::commBuffer[1] = realLength / 256;
	Z21Throttle::commBuffer[2] = inHeader % 256;
	Z21Throttle::commBuffer[3] = inHeader / 256;
	Z21Throttle::commBuffer[4] = inXHeader;
	memcpy(Z21Throttle::commBuffer + 5, inpData, inLengthData);

	if (!inXorInData) {    // if xor byte not included in data, compute and write it !
		byte xxor = inXHeader;

		for (unsigned int i = 0; i < inLengthData; i++)
			xxor ^= inpData[i];

		Z21Throttle::commBuffer[inLengthData + 5] = xxor;
	}

	write(Z21Throttle::commBuffer, realLength);

	return true;
}

// sizes : [       2        ][       2        ][   1   ][ 1 ][inLengthData]
// bytes : [length1, length2][Header1, Header2][XHeader][DB0][Data........]
bool Z21Throttle::notify(unsigned int inHeader, unsigned int inXHeader, byte inDB0, byte* inpData, unsigned int inLengthData, bool inXorInData) {
	int realLength = (inLengthData + 6 + (inXorInData == false ? 1 : 0));

	Z21Throttle::commBuffer[0] = realLength % 256;
	Z21Throttle::commBuffer[1] = realLength / 256;
	Z21Throttle::commBuffer[2] = inHeader % 256;
	Z21Throttle::commBuffer[3] = inHeader / 256;
	Z21Throttle::commBuffer[4] = inXHeader;
	Z21Throttle::commBuffer[5] = inDB0;
	memcpy(Z21Throttle::commBuffer + 6, inpData, inLengthData);

	if (!inXorInData) {   // if xor byte not included in data, compute and write it !
		byte xxor = inXHeader^inDB0;

		for (unsigned int i = 0; i < inLengthData; i++)
			xxor ^= inpData[i];

		Z21Throttle::commBuffer[inLengthData + 6] = xxor;
	}

	write(Z21Throttle::commBuffer, realLength);

	return true;
}

void Z21Throttle::notifyStatus() {
	Z21Throttle::replyBuffer[0] = 0;	// main current 1
	Z21Throttle::replyBuffer[1] = 0; // main current 2
	Z21Throttle::replyBuffer[2] = 0; // prog current 1
	Z21Throttle::replyBuffer[3] = 0; // prog current 2
	Z21Throttle::replyBuffer[4] = 0; // filtered main current 1
	Z21Throttle::replyBuffer[5] = 0; // filtered main current 2
	Z21Throttle::replyBuffer[6] = 0; // Temperature 1
	Z21Throttle::replyBuffer[7] = 0; // Temperature 2
	Z21Throttle::replyBuffer[8] = 5; // Supply voltage 1
	Z21Throttle::replyBuffer[9] = 0; // supply voltage 2
	Z21Throttle::replyBuffer[10] = 16; // VCC voltage 1 
	Z21Throttle::replyBuffer[11] = 0; // VCC voltage 2
	Z21Throttle::replyBuffer[12] = 0b00000000;	// CentralState 
	Z21Throttle::replyBuffer[13] = 0b00000000; // CentralStateEx
	Z21Throttle::replyBuffer[14] = 0;
	Z21Throttle::replyBuffer[15] = 0;
	notify(HEADER_LAN_SYSTEMSTATE, Z21Throttle::replyBuffer, 16, true);
}

int Z21Throttle::getOrAddLoco(int cab) {
	int loco = 0;
	for (; loco < MAX_MY_LOCO; loco++) {
		if (myLocos[loco].throttle != '\0' && myLocos[loco].cab == cab) 
			return loco;
	}

	if (loco >= MAX_MY_LOCO) {
		//use first empty "slot" on this client's list, will be added to DCC registration list
		for (int locoToAdd = 0; locoToAdd < MAX_MY_LOCO; locoToAdd++) {
			if (myLocos[locoToAdd].throttle == '\0') { 
				myLocos[locoToAdd].throttle = '0' + this->clientid;
				myLocos[locoToAdd].cab = cab; 
				myLocos[locoToAdd].functionMap = DCC::getFunctionMap(cab); 
				myLocos[locoToAdd].broadcastPending = true; // means speed/dir will be sent later
				mostRecentCab = cab;
				myLocos[locoToAdd].functionToggles = 1<<2; // F2 (HORN)  is a non-toggle
				return locoToAdd;
			}
		}
	}

	return -1;  // no loco found, and no place to add one !
}

void Z21Throttle::notifyLocoInfo(byte inMSB, byte inLSB) {
	int locoAddress = ((inMSB & 0x3F) << 8) + inLSB;
	int loco = getOrAddLoco(locoAddress);

	if (loco == -1)
	    return; //'Too many locos !'

	Z21Throttle::replyBuffer[0] = inMSB;	// loco address msb
	Z21Throttle::replyBuffer[1] = inLSB; // loco address lsb
	Z21Throttle::replyBuffer[2] = B00000100; // 0000CKKK	 C = already controlled    KKK = speed steps 000:14, 010:28, 100:128
	Z21Throttle::replyBuffer[3] = DCC::getThrottleSpeed(locoAddress); // RVVVVVVV  R = forward    VVVVVVV = speed
	if (DCC::getThrottleDirection(locoAddress)) bitSet(Z21Throttle::replyBuffer[3], 7);

	Z21Throttle::replyBuffer[4] = B00000000; // 0DSLFGHJ  D = double traction    S = Smartsearch   L = F0   F = F4   G = F3   H = F2   J = F1
	if (DCC::getFn(locoAddress, 0)) bitSet(Z21Throttle::replyBuffer[4], 4);
	if (DCC::getFn(locoAddress, 1)) bitSet(Z21Throttle::replyBuffer[4], 0);
	if (DCC::getFn(locoAddress, 2)) bitSet(Z21Throttle::replyBuffer[4], 1);
	if (DCC::getFn(locoAddress, 3)) bitSet(Z21Throttle::replyBuffer[4], 2);
	if (DCC::getFn(locoAddress, 4)) bitSet(Z21Throttle::replyBuffer[4], 3);

	Z21Throttle::replyBuffer[5] = B00000000;	// function F5 to F12    F5 is bit0
	if (DCC::getFn(locoAddress, 5)) bitSet(Z21Throttle::replyBuffer[5], 0);
	if (DCC::getFn(locoAddress, 6)) bitSet(Z21Throttle::replyBuffer[5], 1);
	if (DCC::getFn(locoAddress, 7)) bitSet(Z21Throttle::replyBuffer[5], 2);
	if (DCC::getFn(locoAddress, 8)) bitSet(Z21Throttle::replyBuffer[5], 3);
	if (DCC::getFn(locoAddress, 9)) bitSet(Z21Throttle::replyBuffer[5], 4);
	if (DCC::getFn(locoAddress, 10)) bitSet(Z21Throttle::replyBuffer[5],5);
	if (DCC::getFn(locoAddress, 11)) bitSet(Z21Throttle::replyBuffer[5],6);
	if (DCC::getFn(locoAddress, 12)) bitSet(Z21Throttle::replyBuffer[5],7);

	Z21Throttle::replyBuffer[6] = B00000000;	// function F13 to F20   F13 is bit0
	if (DCC::getFn(locoAddress, 13)) bitSet(Z21Throttle::replyBuffer[6], 0);
	if (DCC::getFn(locoAddress, 14)) bitSet(Z21Throttle::replyBuffer[6], 1);
	if (DCC::getFn(locoAddress, 15)) bitSet(Z21Throttle::replyBuffer[6], 2);
	if (DCC::getFn(locoAddress, 16)) bitSet(Z21Throttle::replyBuffer[6], 3);
	if (DCC::getFn(locoAddress, 17)) bitSet(Z21Throttle::replyBuffer[6], 4);
	if (DCC::getFn(locoAddress, 18)) bitSet(Z21Throttle::replyBuffer[6], 5);
	if (DCC::getFn(locoAddress, 19)) bitSet(Z21Throttle::replyBuffer[6], 6);
	if (DCC::getFn(locoAddress, 20)) bitSet(Z21Throttle::replyBuffer[6], 7);

	Z21Throttle::replyBuffer[7] = B00000000;	// function F21 to F28   F21 is bit0
	if (DCC::getFn(locoAddress, 21)) bitSet(Z21Throttle::replyBuffer[7], 0);
	if (DCC::getFn(locoAddress, 22)) bitSet(Z21Throttle::replyBuffer[7], 1);
	if (DCC::getFn(locoAddress, 23)) bitSet(Z21Throttle::replyBuffer[7], 2);
	if (DCC::getFn(locoAddress, 24)) bitSet(Z21Throttle::replyBuffer[7], 3);
	if (DCC::getFn(locoAddress, 25)) bitSet(Z21Throttle::replyBuffer[7], 4);
	if (DCC::getFn(locoAddress, 26)) bitSet(Z21Throttle::replyBuffer[7], 5);
	if (DCC::getFn(locoAddress, 27)) bitSet(Z21Throttle::replyBuffer[7], 6);
	if (DCC::getFn(locoAddress, 28)) bitSet(Z21Throttle::replyBuffer[7], 7);
  
	notify(HEADER_LAN_XPRESS_NET, LAN_X_HEADER_LOCO_INFO, Z21Throttle::replyBuffer, 8, false);
}

void Z21Throttle::notifyTurnoutInfo(byte inMSB, byte inLSB) {
	Z21Throttle::replyBuffer[0] = inMSB;	// turnout address msb
	Z21Throttle::replyBuffer[1] = inLSB; // turnout address lsb
	Z21Throttle::replyBuffer[2] = B00000000; // 000000ZZ	 ZZ : 00 not switched   01 pos1  10 pos2  11 invalid
	notify(HEADER_LAN_XPRESS_NET, LAN_X_HEADER_TURNOUT_INFO, Z21Throttle::replyBuffer, 3, false);
}

void Z21Throttle::notifyLocoMode(byte inMSB, byte inLSB) {
	Z21Throttle::replyBuffer[0] = inMSB;	// loco address msb
	Z21Throttle::replyBuffer[1] = inLSB; // loco address lsb
	Z21Throttle::replyBuffer[2] = B00000000; // 00000000	DCC   00000001 MM
	notify(HEADER_LAN_GET_LOCOMODE, Z21Throttle::replyBuffer, 3, true);
}

void Z21Throttle::notifyFirmwareVersion() {
	Z21Throttle::replyBuffer[0] = 0x01;	// Version major in BCD
	Z21Throttle::replyBuffer[1] = 0x23;	// Version minor in BCD
	notify(HEADER_LAN_XPRESS_NET, LAN_X_HEADER_FIRMWARE_VERSION, 0x0A, Z21Throttle::replyBuffer, 2, false);
}

void Z21Throttle::notifyHWInfo() {
	Z21Throttle::replyBuffer[0] = 0x00;	// Hardware type in BCD on int32
	Z21Throttle::replyBuffer[1] = 0x02;	// Hardware type in BCD on int32
	Z21Throttle::replyBuffer[2] = 0x00;	// Hardware type in BCD on int32
	Z21Throttle::replyBuffer[3] = 0x00;	// Hardware type in BCD on int32
	Z21Throttle::replyBuffer[4] = 0x23;	// Firmware version in BCD on int32
	Z21Throttle::replyBuffer[5] = 0x01;	// Firmware version in BCD on int32
	Z21Throttle::replyBuffer[6] = 0x00;	// Firmware version in BCD on int32
	Z21Throttle::replyBuffer[7] = 0x00;	// Firmware version in BCD on int32
	notify(HEADER_LAN_GET_HWINFO, Z21Throttle::replyBuffer, 8, true);
}

void Z21Throttle::notifyCvNACK(int inCvAddress) {
	Z21Throttle::replyBuffer[0] = highByte(inCvAddress); // cv address msb
	Z21Throttle::replyBuffer[1] = lowByte(inCvAddress); // cv address lsb
	notify(HEADER_LAN_XPRESS_NET, LAN_X_HEADER_CV_NACK, LAN_X_DB0_CV_NACK, Z21Throttle::replyBuffer, 0, false);
}

void Z21Throttle::notifyCvRead(int inCvAddress, int inValue) {
	Z21Throttle::replyBuffer[0] = highByte(inCvAddress); // cv address msb
	Z21Throttle::replyBuffer[1] = lowByte(inCvAddress); // cv address lsb
	Z21Throttle::replyBuffer[2] = inValue; // cv value
	notify(HEADER_LAN_XPRESS_NET, LAN_X_HEADER_CV_RESULT, 0x14, Z21Throttle::replyBuffer, 3, false);
}

void Z21Throttle::setSpeed(byte inNbSteps, byte inDB1, byte inDB2, byte inDB3) {
	bool isForward = bitRead(inDB3, 7);
	byte speed = inDB3;
	bitClear(speed, 7);

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : speed %d"), clientid, speed * (isForward ? 1:-1));

	int locoAddress = ((inDB1 & 0x3F) << 8) + inDB2;

	if (getOrAddLoco(locoAddress) == -1)
    	return;

	DCC::setThrottle(locoAddress, speed, isForward);

	if ((this->broadcastFlags & BROADCAST_BASE) != 0)
		notifyLocoInfo(inDB1, inDB2);
}

//
// TODO Pass through a text message to avoid multi thread locks...
//

void Z21Throttle::setFunction(byte inDB1, byte inDB2, byte inDB3) {
	// inDB3 :  TTNN NNNN		TT:00 off, TT:01 on; TT:10 toggle   NNNNNN function number

	byte action = bitRead(inDB3, 6) + 2 * bitRead(inDB3, 7);
	byte function = inDB3;
	bitClear(function, 6);
	bitClear(function, 7);
	bool activeFlag = action == 0b01;

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : function %d %s"), clientid, function, activeFlag?"ON":"OFF");

	int locoAddress = ((inDB1 & 0x3F) << 8) + inDB2;
	if (getOrAddLoco(locoAddress) == -1)
    	return;

	if (action == 0b10)	{	// toggle
		bool isActivated = DCC::getFn(locoAddress, function);
		activeFlag = !isActivated;
	}

	DCC::setFn(locoAddress, function, activeFlag);
	if ((this->broadcastFlags & BROADCAST_BASE) != 0)
		notifyLocoInfo(inDB1, inDB2);
}

//
// TODO Pass through a text message to avoid multi thread locks...
//

void Z21CvValueCallback(int16_t inValue)
{
	Z21Throttle::cvValue = inValue;

	if (inValue == -1)
		Z21Throttle::readWriteThrottle->notifyCvNACK(Z21Throttle::cvAddress);
	else
		Z21Throttle::readWriteThrottle->notifyCvRead(Z21Throttle::cvAddress, inValue);

	Z21Throttle::readWriteThrottle = NULL;
}

void Z21Throttle::cvReadProg(byte inDB1, byte inDB2) {
	if (Z21Throttle::readWriteThrottle != NULL)
		return;

	int cvAddress = ((inDB1 & 0x3F) << 8) + inDB2 + 1;

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : cvRead Prog %d"), clientid, cvAddress);

	Z21Throttle::readWriteThrottle = this;
	Z21Throttle::cvAddress = cvAddress - 1;

	void (*ptr)(int16_t) = &Z21CvValueCallback;
	DCC::readCV(cvAddress, ptr);
}

// Working as cvReadProg for the moment...
void Z21Throttle::cvReadMain(byte inDB1, byte inDB2) {
	if (Z21Throttle::readWriteThrottle != NULL)
		return;

	int cvAddress = ((inDB1 & 0x3F) << 8) + inDB2 + 1;

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : cvRead Main cv %d"), clientid, cvAddress);

	Z21Throttle::readWriteThrottle = this;
	Z21Throttle::cvAddress = cvAddress - 1;

	void (*ptr)(int16_t) = &Z21CvValueCallback;
	DCC::readCV(cvAddress, ptr);
}

//
// TODO Pass through a text message to avoid multi thread locks...
//

void Z21Throttle::cvWriteProg(byte inDB1, byte inDB2, byte inDB3) {
	if (Z21Throttle::readWriteThrottle != NULL)
		return;

	int cvAddress = ((inDB1 & 0x3F) << 8) + inDB2 + 1;

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : cvWrite Prog cv %d value %d"), clientid, cvAddress, inDB3);

	Z21Throttle::readWriteThrottle = this;
	Z21Throttle::cvAddress = cvAddress - 1;

	void (*ptr)(int16_t) = &Z21CvValueCallback;
	DCC::writeCVByte(cvAddress, inDB3, ptr);
}

// Working as cvReadProg for the moment...
void Z21Throttle::cvReadPom(byte inDB1, byte inDB2, byte inDB3, byte inDB4) {
	if (Z21Throttle::readWriteThrottle != NULL)
		return;

	int locoAddress = ((inDB1 & 0x3F) << 8) + inDB2;
	int cvAddress = ((inDB3 & B00000011) << 8) + inDB4 + 1;

	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : cvRead Pom Loco %d cv %d"), clientid, locoAddress, cvAddress);

	Z21Throttle::readWriteThrottle = this;
	Z21Throttle::cvAddress = cvAddress - 1;

	void (*ptr)(int16_t) = &Z21CvValueCallback;
	DCC::readCV(cvAddress, ptr);
}

bool Z21Throttle::parse() {
	bool done = false;
	byte DB[100];
	CircularBuffer* pBuffer = clientsUDP[this->clientid].pudpBuffer;

	if (pBuffer == NULL)
		return false;
	if (pBuffer->isEmpty())
		return false;

	int lengthData = pBuffer->GetInt16() - 4;	// length of the data = total length - length of length (!) - length of header
	int header = pBuffer->GetInt16();
	byte Xheader = 0;
	byte DB0 = 0;
	int nbLocos = CountLocos();

	if (lengthData > 0)	{
		pBuffer->GetBytes(DB, lengthData);
		if (Diag::Z21THROTTLEDATA) DIAG(F("%d <- len:%d  header:0x%02x  : 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x"),
					this->clientid, lengthData, header,
					(lengthData > 0)?DB[0]:0,
					(lengthData > 1)?DB[1]:0,
					(lengthData > 2)?DB[2]:0,
					(lengthData > 3)?DB[3]:0,
					(lengthData > 4)?DB[4]:0,
					(lengthData > 5)?DB[5]:0,
					(lengthData > 6)?DB[6]:0,
					(lengthData > 7)?DB[7]:0,
					(lengthData > 8)?DB[8]:0,
					(lengthData > 9)?DB[9]:0);
	}

	switch (header)	{
		case HEADER_LAN_XPRESS_NET:
			Xheader = DB[0];
			switch (Xheader) {
				case LAN_X_HEADER_GENERAL:
					DB0 = DB[1];
					switch (DB0) {
					case LAN_X_DB0_GET_VERSION:
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d GET_VERSION"), this->clientid);
						break;
					case LAN_X_DB0_GET_STATUS:
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d GET_STATUS  "), this->clientid);
						notifyStatus();
						done = true;
						break;
					case LAN_X_DB0_SET_TRACK_POWER_OFF:
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d POWER_OFF"), this->clientid);
						//
						// TODO Pass through a text message to avoid multi thread locks...
						//
						TrackManager::setMainPower(POWERMODE::OFF);
						done = true;
						break;
					case LAN_X_DB0_SET_TRACK_POWER_ON:
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d POWER_ON"), this->clientid);
						//
						// TODO Pass through a text message to avoid multi thread locks...
						//
						TrackManager::setMainPower(POWERMODE::ON);
						done = true;
						break;
					}
					break;
				case LAN_X_HEADER_SET_STOP:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d EMERGENCY_STOP"), this->clientid);
					//
					// TODO Pass through a text message to avoid multi thread locks...
					//
					//Emergency Stop  (speed code 1)
					// setThrottle will cause a broadcast so notification will be sent
					LOOPLOCOS('*', 0) { DCC::setThrottle(myLocos[loco].cab, 1, DCC::getThrottleDirection(myLocos[loco].cab)); }
					done = true;
					break;
				case LAN_X_HEADER_SET_LOCO:
					DB0 = DB[1];
					switch (DB0) {
						case LAN_X_DB0_LOCO_DCC14:
							if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOCO DCC 14 SPEED"), this->clientid);
							setSpeed(14, DB[2], DB[3], DB[4]);
							done = true;
							break;
						case LAN_X_DB0_LOCO_DCC28:
							if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOCO DCC 28 SPEED"), this->clientid);
							setSpeed(28, DB[2], DB[3], DB[4]);
							done = true;
							break;
						case LAN_X_DB0_LOCO_DCC128:
							if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOCO DCC 128 SPEED"), this->clientid);
							setSpeed(128, DB[2], DB[3], DB[4]);
							done = true;
							break;
						case LAN_X_DB0_SET_LOCO_FUNCTION:
							if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOCO DCC FUNCTION"), this->clientid);
							setFunction(DB[2], DB[3], DB[4]);
							if (Diag::Z21THROTTLE) {
								// Debug capacity to print data...
								byte function = DB[4];
								bitClear(function, 6);
								bitClear(function, 7);
								if (function == 12) { // why not ?
									printClientsUDP();
									printThrottles(true);
								}
							}
							done = true;
							break;
					}
					break;
				case LAN_X_HEADER_GET_LOCO_INFO:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOCO INFO: "), this->clientid);
					notifyLocoInfo(DB[2], DB[3]);
					done = true;

					break;
				case LAN_X_HEADER_GET_TURNOUT_INFO:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d TURNOUT INFO  "), this->clientid);
					notifyTurnoutInfo(DB[1], DB[2]);
					done = true;

					break;
				case LAN_X_HEADER_GET_FIRMWARE_VERSION:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d FIRMWARE VERSION  "), this->clientid);
					notifyFirmwareVersion();
					done = true;
					break;
				case LAN_X_HEADER_CV_READ:
					if (TrackManager::getProgDriver() != NULL) {
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d CV READ PROG "), this->clientid);
						// DB0 should be 0x11
						cvReadProg(DB[2], DB[3]);
					}
					else {
						//
						// TODO Dont work today...
						//

						// If no prog track, read on the main track !
						if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d CV READ MAIN "), this->clientid);
						// DB0 should be 0x11
						cvReadMain(DB[2], DB[3]);
					}
					done = true;
					break;
				case LAN_X_HEADER_CV_POM:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d CV READ POM"), this->clientid);
					// DB0 should be 0x11
					cvReadPom(DB[2], DB[3], DB[4], DB[5]);
					done = true;
					break;
				case LAN_X_HEADER_CV_WRITE:
					if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d CV WRITE "), this->clientid);
					notifyFirmwareVersion();
					done = true;
					break;
				case LAN_X_HEADER_SET_TURNOUT:
				case 0x22:
					break;
			}
			break;

		case HEADER_LAN_SET_BROADCASTFLAGS:
			this->broadcastFlags = CircularBuffer::GetInt32(DB, 0);
			if (Diag::Z21THROTTLEDATA) DIAG(F("BROADCAST FLAGS %d : %s %s %s %s %s %s %s %s %s %s %s"), this->clientid,
							(this->broadcastFlags & BROADCAST_BASE)	? "BASE " : "" ,
							(this->broadcastFlags & BROADCAST_RBUS)	? "RBUS " : "" ,
							(this->broadcastFlags & BROADCAST_RAILCOM)	? "RAILCOM " : "" ,
							(this->broadcastFlags & BROADCAST_SYSTEM)	? "SYSTEM " : "" ,
							(this->broadcastFlags & BROADCAST_BASE_LOCOINFO)	? "LOCOINFO " : "" ,
							(this->broadcastFlags & BROADCAST_LOCONET)	? "LOCONET " : "" ,
							(this->broadcastFlags & BROADCAST_LOCONET_LOCO)	? "LOCONET_LOCO " : "" ,
							(this->broadcastFlags & BROADCAST_LOCONET_SWITCH)	? "LOCONET_SWITCH " : "" ,
							(this->broadcastFlags & BROADCAST_LOCONET_DETECTOR)	? "LOCONET_DETECTOR " : "" ,
							(this->broadcastFlags & BROADCAST_RAILCOM_AUTO)	? "RAILCOM_AUTO " : "" ,
							(this->broadcastFlags & BROADCAST_CAN)	? "CAN" : "" );
			done = true;
			break;
		case HEADER_LAN_GET_LOCOMODE:
			if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d GET LOCOMODE"), this->clientid);
			notifyLocoMode(DB[0], DB[1]);	// big endian here, but resend the same as received, so no problem.
			done = true;
			break;

		case HEADER_LAN_SET_LOCOMODE:
			if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d SET LOCOMODE"), this->clientid);
			done = true;
			break;
		case HEADER_LAN_GET_HWINFO:
			if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d GET HWINFO"), this->clientid);
			notifyHWInfo();	// big endian here, but resend the same as received, so no problem.
			done = true;
			break;
		case HEADER_LAN_LOGOFF:
			if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d LOGOFF"), this->clientid);
			this->clientid = -1;
			done = true;
			break;
		case HEADER_LAN_SYSTEMSTATE_GETDATA:
			if (Diag::Z21THROTTLEVERBOSE) DIAG(F("%d SYSTEMSTATE GETDATA"), this->clientid);
			notifyStatus();	// big endian here, but resend the same as received, so no problem.
			done = true;
			break;
		case HEADER_LAN_GET_SERIAL_NUMBER:
		case HEADER_LAN_GET_BROADCASTFLAGS:
		case HEADER_LAN_GET_TURNOUTMODE:
		case HEADER_LAN_SET_TURNOUTMODE:
		case HEADER_LAN_RMBUS_DATACHANGED:
		case HEADER_LAN_RMBUS_GETDATA:
		case HEADER_LAN_RMBUS_PROGRAMMODULE:
		case HEADER_LAN_RAILCOM_DATACHANGED:
		case HEADER_LAN_RAILCOM_GETDATA:
		case HEADER_LAN_LOCONET_DISPATCH_ADDR:
		case HEADER_LAN_LOCONET_DETECTOR:
			break;
	}

	if (!done) {
    	if (Diag::Z21THROTTLE) DIAG(F("Z21 Throttle %d : not treated :  header:%x   Xheader:%x   DB0:%x"), this->clientid, header, Xheader, DB0);
	}
	else {
		int newNbLocos = CountLocos();
		if (nbLocos != newNbLocos)
			printLocomotives();
	}
	return true;
}
