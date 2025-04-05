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
#ifndef KeywordHasher_h
#define KeywordHasher_h

#include <Arduino.h>
constexpr uint16_t CompiletimeKeywordHasher(const char* sv, uint16_t running = 0) {
  return (*sv == 0) ? running
                    : CompiletimeKeywordHasher(sv + 1,
                                               (*sv >= '0' && *sv <= '9') ? (10 * running + *sv - '0')          // Numeric hash
                                                                          : ((running << 5) + running) ^ *sv);  //
}

constexpr int16_t operator""_hk(const char* keyword, size_t len) {
  return (int16_t)CompiletimeKeywordHasher(keyword, len * 0);
}

/* Some historical values for testing:
const int16_t HASH_KEYWORD_MAIN = 11339;
const int16_t HASH_KEYWORD_SLOW = -17209;
const int16_t HASH_KEYWORD_SPEED28 = -17064;
const int16_t HASH_KEYWORD_SPEED128 = 25816;
*/

static_assert("MAIN"_hk == 11339, "Keyword hasher error");
static_assert("SLOW"_hk == -17209, "Keyword hasher error");
static_assert("SPEED28"_hk == -17064, "Keyword hasher error");
static_assert("SPEED128"_hk == 25816, "Keyword hasher error");

// Compile time converter from "abcd"_s7  to the 7 segment nearest equivalent

constexpr uint8_t seg7Digits[] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111,  // 0..3
    0b01100110, 0b01101101, 0b01111101, 0b00000111,  // 4..7
    0b01111111, 0b01101111                           // 8..9
};

constexpr uint8_t seg7Letters[] = {
    0b01110111, 0b01111100, 0b00111001, 0b01011110,  // ABCD
    0b01111001, 0b01110001, 0b00111101, 0b01110110,  // EFGH
    0b00000100, 0b00011110, 0b01110010, 0b00111000,  // IJKL
    0b01010101, 0b01010100, 0b01011100, 0b01110011,  // MNOP
    0b10111111, 0b01010000, 0b01101101, 0b01111000,  // QRST
    0b00111110, 0b00011100, 0b01101010, 0b01001001,  // UVWX
    0b01100110, 0b01011011                           // YZ
};
constexpr uint8_t seg7Space = 0b00000000;
constexpr uint8_t seg7Minus = 0b01000000;
constexpr uint8_t seg7Equals = 0b01001000;

constexpr uint32_t CompiletimeSeg7(const char* sv, uint32_t running, size_t rlen) {
  return (*sv == 0 || rlen == 0) ? running << (8 * rlen)
                                 : CompiletimeSeg7(sv + 1,
                                                   (*sv >= '0' && *sv <= '9')   ? (running << 8) | seg7Digits[*sv - '0']
                                                   : (*sv >= 'A' && *sv <= 'Z') ? (running << 8) | seg7Letters[*sv - 'A']
                                                   : (*sv >= 'a' && *sv <= 'z') ? (running << 8) | seg7Letters[*sv - 'a']
                                                   : (*sv == '-')               ? (running << 8) | seg7Minus
                                                   : (*sv == '=')               ? (running << 8) | seg7Equals
                                                                                : (running << 8) | seg7Space,
                                                   rlen - 1);  //
}

constexpr uint32_t operator""_s7(const char* keyword, size_t len) {
  return CompiletimeSeg7(keyword, 0 * len, 4);
}
#endif
