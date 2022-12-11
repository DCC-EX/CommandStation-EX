/*
 *  © 2022 Peter Cole
 *
 *  This file is part of EX-CommandStation
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

/*
*   This file defines default pin maps for the various architectures supported
*   by default by EX-IOExpander.
*   
*   Custom user defined pinmaps should be defined in myEX-IOExpander.h.
*
*   Any modifications to this file will be overwritten by future software updates.
*/

// #define DEFAULT_NANO_DIGITAL_PINMAP 2,3,4,5,6,7,8,9,10,11,12,13
// #define DEFAULT_NANO_ANALOGUE_PINMAP A0,A1,A2,A3,A6,A7

#define EXIO_UNO_DIGITAL_PINS 12
#define EXIO_UNO_ANALOGUE_PINS 4

#define EXIO_NANO_DIGITAL_PINS 12
#define EXIO_NANO_ANALOGUE_PINS 6

#define EXIO_MEGA_DIGITAL_PINS 46
#define EXIO_MEGA_ANALOGUE_PINS 16

// #define EXIO_UNO_DIGITAL_PINMAP 2,3,4,5,6,7,8,9,10,11,12,13
// #define EXIO_UNO_ANALOGUE_PINMAP A0,A1,A2,A3

// #define DEFAULT_MEGA_DIGITAL_PINMAP 2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,22,23,24,25,26,27,28,29, \
//                                     30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49
// #define DEFAULT_MEGA_ANALOGUE_PINMAP A0,A1,A2,A3,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
