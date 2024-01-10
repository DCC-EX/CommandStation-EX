/*
 *  © 2024 Vincent Hamp and Chris Harlow
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


/* Reader be aware: 
 This function implements the _hk data type so that a string keyword
 is hashed to the same value as the DCCEXParser uses to hash incoming 
 keywords. 
 Thus  "MAIN"_hk  generates exactly the same run time vakue 
 as   const int16_t HASH_KEYWORD_MAIN=11339  
*/
#ifndef KeywordHAsher_h
#define KeywordHasher_h

#include <Arduino.h>
constexpr uint16_t CompiletimeKeywordHasher(const char *  sv, uint16_t running=0) {
    return (*sv==0) ? running : CompiletimeKeywordHasher(sv+1,
          (*sv >= '0' && *sv <= '9') 
            ? (10*running+*sv-'0')  // Numeric hash  
            : ((running << 5) + running) ^ *sv
            );  // 
}

constexpr int16_t operator""_hk(const char * keyword, size_t len) 
{
    return (int16_t) CompiletimeKeywordHasher(keyword,len*0);
}

/* Some historical values for testing:
const int16_t HASH_KEYWORD_MAIN = 11339;
const int16_t HASH_KEYWORD_SLOW = -17209;
const int16_t HASH_KEYWORD_SPEED28 = -17064;
const int16_t HASH_KEYWORD_SPEED128 = 25816;
*/

static_assert("MAIN"_hk == 11339,"Keyword hasher error");
static_assert("SLOW"_hk == -17209,"Keyword hasher error");
static_assert("SPEED28"_hk == -17064,"Keyword hasher error");
static_assert("SPEED128"_hk == 25816,"Keyword hasher error");
#endif