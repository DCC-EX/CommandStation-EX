/*
 *  © 2025 Chris Harlow
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

// Count the number of arguments
#define FOR_EACH_NARG(...) FOR_EACH_NARG_HELPER(__VA_ARGS__,8,7, 6,5,4, 3, 2, 1, 0)
#define FOR_EACH_NARG_HELPER(_1, _2, _3, _4, _5, _6, _7, _8, N, ...) N

// Step 2: Force proper expansion (extra indirection to resolve `##`)
#define EXPAND(x) x
#define CONCAT(a, b) a##b


#define ZZZ(_i,_arg) \
  if ( #_arg[0]<='Z' && p[_i]!=CONCAT(#_arg,_hk)) break; \
  auto _arg=p[_i]; (void) _arg; 

// Each ZZ terminates the previous one 
#define ZPREP(op,count) return true; } if (opcode==#op[0] && params==count) for (;;) {
#define Z1(op)                      ZPREP(op,0)
#define Z2(op,_1)                   ZPREP(op,1) ZZZ(0,_1) 
#define Z3(op,_1,_2)                ZPREP(op,2) ZZZ(0,_1) ZZZ(1,_2)
#define Z4(op,_1,_2,_3)             ZPREP(op,3) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3)
#define Z5(op,_1,_2,_3,_4)          ZPREP(op,4) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4)
#define Z6(op,_1,_2,_3,_4,_5)       ZPREP(op,5) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5)
#define Z7(op,_1,_2,_3,_4,_5,_6)    ZPREP(op,6) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5) ZZZ(5,_6)
#define Z8(op,_1,_2,_3,_4,_5,_6,_7) ZPREP(op,7) ZZZ(0,_1) ZZZ(1,_2) ZZZ(2,_3) ZZZ(3,_4) ZZZ(4,_5) ZZZ(5,_6) ZZZ(6,_7)

#define ZRIP(count) CONCAT(Z,count)

#define ZC1(op)                      
#define ZC2(op,_1)                   ZZCHK(0,_1) 
#define ZC3(op,_1,_2)                ZZCHK(0,_1) ZZCHK(1,_2)
#define ZC4(op,_1,_2,_3)             ZZCHK(0,_1) ZZCHK(1,_2) ZZCHK(2,_3)
#define ZC5(op,_1,_2,_3,_4)          ZZCHK(0,_1) ZZCHK(1,_2) ZZCHK(2,_3) ZZCHK(3,_4)
#define ZC6(op,_1,_2,_3,_4,_5)       ZZCHK(0,_1) ZZCHK(1,_2) ZZCHK(2,_3) ZZCHK(3,_4) ZZCHK(4,_5)
#define ZC7(op,_1,_2,_3,_4,_5,_6)    ZZCHK(0,_1) ZZCHK(1,_2) ZZCHK(2,_3) ZZCHK(3,_4) ZZCHK(4,_5) ZZCHK(5,_6)
#define ZC8(op,_1,_2,_3,_4,_5,_6,_7) ZZCHK(0,_1) ZZCHK(1,_2) ZZCHK(2,_3) ZZCHK(3,_4) ZZCHK(4,_5) ZZCHK(5,_6) ZZCHK(6,_7)
#define ZCRIP(count) CONCAT(ZC,count)

#define ZZ(...) \
  ZRIP(FOR_EACH_NARG(__VA_ARGS__))(__VA_ARGS__) \
  DCCEXParser::matchedCommandFormat = F( #__VA_ARGS__); \
  ZCRIP(FOR_EACH_NARG(__VA_ARGS__))(__VA_ARGS__)
  

 
#define ZZBEGIN if (false) {
#define ZZEND return true; } return false;
#define CHECK(x,...) if (!(x)) { DCCEXParser::checkFailedFormat=#__VA_ARGS__[0]?F(#__VA_ARGS__):F(#x); return false;}
#define REPLY(format,...) StringFormatter::send(stream,F(format), ##__VA_ARGS__);
#define EXPECT_CALLBACK CHECK(stashCallback(stream, p, ringStream))
// helper macro to hide command from documentation extractor
#define ZZ_nodoc ZZ

#define ZCHECK(_checkname,_index,_pname,_min,_max) \
  if (CONCAT(#_pname,_hk) == CONCAT(#_checkname,_hk) \
        && (p[_index]<_min || p[_index]>_max)) CHECK(false,_checkname _min .. _max) 

// Automatic range checks based on name of inserted parameter
#define ZZCHK(_index,_pname)\
ZCHECK(loco,_index,_pname,0,10239) \
ZCHECK(track,_index,_pname,'A','H') \
ZCHECK(cv,_index,_pname,1,255) \
ZCHECK(value,_index,_pname,0,255) \
ZCHECK(bit,_index,_pname,0,7) \
ZCHECK(bitvalue,_index,_pname,0,1)

#define ZGETSTRING(p) ((char *)(com+ (p & 0x00FF)))



  


