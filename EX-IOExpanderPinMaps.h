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

#define DEFAULT_NANO_DIGITAL_PINMAP new EXIODigitalPinMap(12,2,3,4,5,6,7,8,9,10,11,12,13,A0,A1,A2,A3,A6,A7)
#define DEFAULT_NANO_ANALOGUE_PINMAP new EXIOAnaloguePinMap(6,A0,A1,A2,A3,A6,A7)

#define DEFAULT_UNO F("DEFAULT_UNO"), \
  new EXIODigitalPinMap(12,2,3,4,5,6,7,8,9,10,11,12,13,A0,A1,A2,A3,A6,A7) \
  new EXIOAnaloguePinMap(4,A0,A1,A2,A3)

#define DEFAULT_MEGA F("DEFAULT_MEGA"), \
  new EXIODigitalPinMap(12,2,3,4,5,6,7,8,9,10,11,12,13,A0,A1,A2,A3,A6,A7) \
  new EXIOAnaloguePinMap(4,A0,A1,A2,A3)