/*
 *  © 2020,Gregor Baues,  Chris Harlow. All rights reserved.
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
#ifndef CommandDistributor_h
#define CommandDistributor_h
#include "DCCEXParser.h"

typedef void (*HTTP_CALLBACK)(Print *stream, byte *cmd);

class CommandDistributor {

public :
  static void setHTTPCallback(HTTP_CALLBACK callback);
  static bool parse(byte clientId,byte* buffer, Print * streamer);


private:
   static HTTP_CALLBACK httpCallback;
   static bool isHTTP(byte * buffer);
   static DCCEXParser * parser;
};

#endif
