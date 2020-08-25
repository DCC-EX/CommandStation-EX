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

#ifndef COMMANDSTATION_COMMINTERFACE_DCCEXPARSER_H_
#define COMMANDSTATION_COMMINTERFACE_DCCEXPARSER_H_

#include "../DCC/DCCMain.h"
#include "../DCC/DCCService.h"

#include <Arduino.h>

struct DCCEXParser
{
  static DCCMain *mainTrack;
  static DCCService *progTrack;
  static void init(DCCMain* mainTrack_, DCCService* progTrack_);
  static void parse(Print* stream, const char *);
  static void cvResponse(Print* stream, serviceModeResponse response);
  static void POMResponse(Print* stream, RailcomPOMResponse response);
  static void trackPowerCallback(const char* name, bool status);
private:
  static int stringParser(const char * com, int result[]);
  static const int MAX_PARAMS=10; 
  static int p[MAX_PARAMS];
};

#endif  // COMMANDSTATION_COMMINTERFACE_DCCEXPARSER_H_