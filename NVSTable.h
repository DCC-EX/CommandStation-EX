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
    static const uint8_t NVS_MAX = 255;
    static const uint16_t NVS_IS_STRING = 0x7FFE; // Special marker for string values
    static void load();
    static void dump(Print * stream);
    static void dump(Print * stream, uint8_t nvsNumber);
    static void setNVS(uint8_t nvsNumber, int16_t value);
    static void setNVS(uint8_t nvsNumber, String value);
    static int16_t getNVS(uint8_t nvsNumber);
    static String getTextNVS(uint8_t nvsNumber);
    
    // streaming for web interface
    static void streamJSArray(Print * stream);
    static void applyChanges(const String& changes);

    // Acess NVS() values from EXRAIL scripts. This will decode a token into its corresponding NVS value.
    // This identifies EXRAIL type 32-bit tokens which may represent NVS[x]+
    static int16_t decodeNVSToken(int32_t token);
    static int16_t getNVSDuringBoot(uint8_t nvsnum);
  private:
    static int16_t nvs[NVS_MAX+1];
    static byte bootNeeded[(NVS_MAX+1)/8+1]; // bit array to track which NVS values need to be set during boot
};

#endif