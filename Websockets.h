/*
 *  © 2023 Chris Harlow
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
#ifndef Websockets_h
#define Websockets_h
#include <Arduino.h>
#include "RingStream.h"
class Websockets {
    public:
    static bool checkConnectionString(byte clientId,byte * cmd, RingStream * outbound );
    static byte * unmask(byte clientId,RingStream *ring, byte * buffer);
    static int16_t getOutboundHeaderSize(uint16_t dataLength);
    static int fillOutboundHeader(uint16_t dataLength, byte * buffer);
    static void writeOutboundHeader(Print * stream,uint16_t dataLength);
    static const byte WEBSOCK_CLIENT_MARKER=0x80;
};

#endif