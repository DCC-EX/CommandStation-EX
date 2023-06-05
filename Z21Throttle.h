/*
 *  © 2023 Thierry Paris / Locoduino
 *  All rights reserved.
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
#ifndef Z21Throttle_h
#define Z21Throttle_h

#include "CircularBuffer.hpp"
#include "WiFiClient.h"

#define UDPBYTE_SIZE	64
#define UDP_BUFFERSIZE	256

struct MYLOCOZ21 {
    char throttle; //indicates which throttle letter on client, '0' + clientid
    int cab; //address of this loco
    bool broadcastPending;
    uint32_t functionMap;
    uint32_t functionToggles;
};

class NetworkClientUDP {
	public:
	NetworkClientUDP() {
		this->pudpBuffer = new CircularBuffer(UDP_BUFFERSIZE);
		this->pudpBuffer->begin(true);
	};
	bool ok() {
		return (inUse);
	};

	bool inUse = true;
	bool connected = false;
	CircularBuffer *pudpBuffer = NULL;
	IPAddress remoteIP;
	int remotePort;

	static WiFiUDP client;
};

class Z21Throttle {
	public:  
		static void loop();
		static Z21Throttle* getOrAddThrottle(int clientId); 
		static void markForBroadcast(int cab);
		static void forget(byte clientId);
		static void findUniqThrottle(int id, char *u);
		static void setup(IPAddress ip, int port);

		void notifyCvNACK(int inCvAddress);
		void notifyCvRead(int inCvAddress, int inValue);
		
		bool parse();

		static Z21Throttle *readWriteThrottle;	// NULL if no throttle is reading or writing a CV...
		static int cvAddress;
		static int cvValue;

	private: 
		Z21Throttle(int clientId);
		~Z21Throttle();

		static const int MAX_MY_LOCO=10;      // maximum number of locos assigned to a single client
		static const int HEARTBEAT_SECONDS=10; // heartbeat at 4secs to provide messaging transport
		static const int ESTOP_SECONDS=20;     // eStop if no incoming messages for more than 8secs
		static Z21Throttle* firstThrottle;
		static byte commBuffer[100];
		static byte replyBuffer[20];
		static char LorS(int cab); 
		static bool isThrottleInUse(int cab);
		static void setSendTurnoutList();
		bool areYouUsingThrottle(int cab);
		Z21Throttle* nextThrottle;

		int clientid;
		char uniq[17] = "";

		MYLOCOZ21 myLocos[MAX_MY_LOCO];   

		int CountLocos() {  
			int count = 0;
			for (int loco=0;loco<MAX_MY_LOCO;loco++)
				if (myLocos[loco].throttle != '\0')
					count++;

			return count;
		}

		bool initSent; // valid connection established
		bool exRailSent; // valid connection established
		uint16_t mostRecentCab;
		int turnoutListHash;  // used to check for changes to turnout list
		bool lastPowerState;  // last power state sent to this client
		int32_t broadcastFlags;

		int getOrAddLoco(int cab);
		void printLocomotives(bool addTab = false);
		static void printThrottles(bool printLocomotives);

		// sizes : [       2        ][       2        ][inLengthData]
		// bytes : [length1, length2][Header1, Header2][Data........]
		bool notify(unsigned int inHeader, byte* inpData, unsigned int inLengthData, bool inXorInData);

		// sizes : [       2        ][       2        ][   1   ][inLengthData]
		// bytes : [length1, length2][Header1, Header2][XHeader][Data........]
		bool notify(unsigned int inHeader, unsigned int inXHeader, byte* inpData, unsigned int inLengthData, bool inXorInData);

		// sizes : [       2        ][       2        ][   1   ][ 1 ][inLengthData]
		// bytes : [length1, length2][Header1, Header2][XHeader][DB0][Data........]
		bool notify(unsigned int inHeader, unsigned int inXHeader, byte inDB0, byte* inpData, unsigned int inLengthData, bool inXorInData);

		void notifyStatus();
		void notifyLocoInfo(byte inMSB, byte inLSB);
		void notifyTurnoutInfo(byte inMSB, byte inLSB);
		void notifyLocoMode(byte inMSB, byte inLSB);
		void notifyFirmwareVersion();
		void notifyHWInfo();
		void write(byte* inpData, int inLengthData);

		void setSpeed(byte inNbSteps, byte inDB1, byte inDB2, byte inDB3);
		void setFunction(byte inDB1, byte inDB2, byte inDB3);
		void cvReadProg(byte inDB1, byte inDB2);
		void cvWriteProg(byte inDB1, byte inDB2, byte inDB3);
		void cvReadMain(byte inDB1, byte inDB2);
		void cvReadPom(byte inDB1, byte inDB2, byte inDB3, byte inDB4);

		// callback stuff to support prog track acquire
		static Z21Throttle * stashInstance;
		static byte         stashClient;
		static char         stashThrottleChar;
		static void         getLocoCallback(int16_t locoid);
};

#define Z21_UDPPORT		21105
#define Z21_TIMEOUT		60000		// if no activity during 1 minute, disconnect the throttle...

#define HEADER_LAN_GET_SERIAL_NUMBER 0x10
#define HEADER_LAN_LOGOFF 0x30
#define HEADER_LAN_XPRESS_NET 0x40
#define HEADER_LAN_SET_BROADCASTFLAGS 0x50
#define HEADER_LAN_GET_BROADCASTFLAGS 0x51
#define HEADER_LAN_SYSTEMSTATE_GETDATA 0x85    //0x141 0x21 0x24 0x05
#define HEADER_LAN_GET_HWINFO 0x1A
#define HEADER_LAN_GET_LOCOMODE 0x60
#define HEADER_LAN_SET_LOCOMODE 0x61
#define HEADER_LAN_GET_TURNOUTMODE 0x70
#define HEADER_LAN_SET_TURNOUTMODE 0x71
#define HEADER_LAN_RMBUS_DATACHANGED 0x80
#define HEADER_LAN_RMBUS_GETDATA 0x81
#define HEADER_LAN_RMBUS_PROGRAMMODULE 0x82
#define HEADER_LAN_RAILCOM_DATACHANGED 0x88
#define HEADER_LAN_RAILCOM_GETDATA 0x89
#define HEADER_LAN_LOCONET_DISPATCH_ADDR 0xA3
#define HEADER_LAN_LOCONET_DETECTOR 0xA4

#define LAN_GET_CONFIG 0x12

#define LAN_X_HEADER_GENERAL 0x21
#define LAN_X_HEADER_SET_STOP 0x80
#define LAN_X_HEADER_GET_FIRMWARE_VERSION 0xF1  //0x141 0x21 0x21 0x00 
#define LAN_X_HEADER_GET_LOCO_INFO 0xE3
#define LAN_X_HEADER_SET_LOCO 0xE4
#define LAN_X_HEADER_GET_TURNOUT_INFO 0x43
#define LAN_X_HEADER_SET_TURNOUT 0x53
#define LAN_X_HEADER_CV_READ 0x23
#define LAN_X_HEADER_CV_WRITE 0x24
#define LAN_X_HEADER_CV_POM 0xE6

#define LAN_X_DB0_GET_VERSION 0x21
#define LAN_X_DB0_GET_STATUS 0x24
#define LAN_X_DB0_SET_TRACK_POWER_OFF 0x80
#define LAN_X_DB0_SET_TRACK_POWER_ON 0x81
#define LAN_X_DB0_LOCO_DCC14	0x10
#define LAN_X_DB0_LOCO_DCC28	0x12
#define LAN_X_DB0_LOCO_DCC128	0x13
#define LAN_X_DB0_SET_LOCO_FUNCTION 0xF8
#define LAN_X_DB0_CV_POM_WRITE 0x30
#define LAN_X_DB0_CV_POM_ACCESSORY_WRITE 0x31

#define LAN_X_OPTION_CV_POM_WRITE_BYTE 0xEC
#define LAN_X_OPTION_CV_POM_WRITE_BIT 0xE8
#define LAN_X_OPTION_CV_POM_READ_BYTE 0xE4

// Replies to the controllers
#define HEADER_LAN_SYSTEMSTATE	0x84

#define LAN_X_HEADER_LOCO_INFO	0xEF
#define LAN_X_HEADER_TURNOUT_INFO 0x43
#define LAN_X_HEADER_FIRMWARE_VERSION 0xF3
#define LAN_X_HEADER_CV_NACK 0x61
#define LAN_X_HEADER_CV_RESULT 0x64

#define LAN_X_DB0_CV_NACK_SC 0x12
#define LAN_X_DB0_CV_NACK 0x13

// Broadcast flags

#define BROADCAST_BASE				0x00000001
#define BROADCAST_RBUS				0x00000002
#define BROADCAST_RAILCOM			0x00000004
#define BROADCAST_SYSTEM			0x00000100
#define BROADCAST_BASE_LOCOINFO		0x00010000
#define BROADCAST_LOCONET			0x01000000
#define BROADCAST_LOCONET_LOCO		0x02000000
#define BROADCAST_LOCONET_SWITCH	0x04000000
#define BROADCAST_LOCONET_DETECTOR	0x08000000
#define BROADCAST_RAILCOM_AUTO		0x00040000
#define BROADCAST_CAN				0x00080000

#endif
