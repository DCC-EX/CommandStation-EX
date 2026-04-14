/* 
*  © 2024 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX
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
#ifndef Stash_h
#define Stash_h
#include <Arduino.h>

class Stash {
  public:
    static void clear(int16_t stash_id);
    static void clearAll();
    static void clearAny(int16_t loco_id);
    static int16_t get(int16_t stash_id);
    static void set(int16_t stash_id, int16_t loco_id);
    static void list(Print * stream, int16_t stash_id=0); // id0 = LIST ALL
  private:
    Stash(int16_t stash_id, int16_t loco_id);
    static Stash* first;
    Stash* next;
    int16_t stashId;   
    int16_t locoId;
};
#endif
