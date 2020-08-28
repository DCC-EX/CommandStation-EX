/*
 *  CommInterfaceSerial.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMINTERFACE_COMMINTERFACESERIAL_H_
#define COMMINTERFACE_COMMINTERFACESERIAL_H_

#include "CommInterface.h"

#include <Arduino.h>
#include <HardwareSerial.h>

class SerialInterface : public CommInterface {
public:
	SerialInterface(HardwareSerial &serial, long baud=115200);
	void process();
	void showConfiguration();
	void showInitInfo();
	Print* getStream() { return &serialStream; }
protected:
	HardwareSerial &serialStream;
	long baud;
	String buffer;
	bool inCommandPayload;
};

#endif	// COMMINTERFACE_COMMINTERFACESERIAL_H_