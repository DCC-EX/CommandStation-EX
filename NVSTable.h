/*
 *  © 2020-2026 Chris Harlow
 *  © 2026 Ross Scanlon
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX API
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

/**********************************************************************

The NVS Table class for DCC-EX Command Station Nodes

**********************************************************************/

#ifndef NVSTable_h
#define NVSTable_h
#include <Arduino.h>
class NVSTable {
  public:
    static void load();
    static void dump(Print * stream);
    static void dump(Print * stream, uint16_t nvsNumber);
    static bool saveNeeded();
    static void save();
    static void setNVS(uint16_t nvsNumber, int16_t value, bool autosave=true);
    static void setNVS(uint16_t nvsNumber, const char * value, bool autosave=true);
    static int16_t getNVS(uint16_t nvsNumber, bool atBoot=false);
    static const char * getTextNVS(uint16_t nvsNumber);
    
    // streaming for web interface
    static void streamJSArray(Print * stream);
    static void applyChanges(char * changes,bool fromNewBoot);

    // Acess NVS() values from EXRAIL scripts. This will decode a token into its corresponding NVS value.
    // This identifies EXRAIL type 32-bit tokens which may represent NVS[x]+
    static int16_t decodeNVSToken(int32_t token, bool atBoot);
};

#endif