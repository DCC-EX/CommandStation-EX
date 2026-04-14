/*
 *  © 2023-2025, Barry Daniel  
 *  © 2025       Chris Harlow
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


#ifndef CamParser_H
#define CamParser_H
#include <Arduino.h>
#include "IODevice.h"

class CamParser {
   public:
   static void parse(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
   static void addVpin(VPIN pin);
   private:
    static bool parseN(Print * stream, byte paramCount, int16_t p[]);
    static VPIN CAMBaseVpin;
    static VPIN CAMVPINS[];
    static int vpcount;
};


#endif