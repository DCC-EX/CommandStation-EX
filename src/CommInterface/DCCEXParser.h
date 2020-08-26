/*
 *  DCCEXParser.h
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMINTERFACE_DCCEXPARSER_H_
#define COMMINTERFACE_DCCEXPARSER_H_

#include "../DCC/DCC.h"

#include <Arduino.h>

struct DCCEXParser
{
  static DCC *mainTrack;
  static DCC *progTrack;
  static void init(DCC* mainTrack_, DCC* progTrack_);
  static void parse(Print* stream, const char *);
  static void cvResponse(Print* stream, serviceModeResponse response);
  static void POMResponse(Print* stream, RailComPOMResponse response);
  static void trackPowerCallback(const char* name, bool status);
private:
  static int stringParser(const char * com, int result[]);
  static const int MAX_PARAMS=10; 
  static int p[MAX_PARAMS];
};

#endif  // COMMINTERFACE_DCCEXPARSER_H_