/*
 *  © 2020, Gregor Baues. All rights reserved.
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

#include <Arduino.h>

// #include "DIAG.h"
// #include "MemoryFree.h"
// #include "NetworkDiag.h"


int _dMem = 0;
int _cMem = 0;
byte _nLogLevel = 0;         // (Third digit)
byte _nInfoLevel = 1;        // (Second digit)runtime level of the details of the messages displayed; 1 = only the diag messagges, 2 - incl time / file / line / freemem information  - TBD
byte _dOutput = 1;           // (First digit) where the diag messages shall be send; 1 = Serial, 2 = future CLI on port 23 - TBD (Hundreds)

// e.g. 124 Send up to TRACE with full info to Serial  or 211 send basic INFO level messages to the CLI

// nLogLevel 0 to 5 send to serial according to the log level 0 = SILENT, 1 = INFO, 2 = WARNING, 3 = ERROR, 4 = TRACE, 5 = DEBUG
// nLoglevel  10 to 15 send to network client (10) +  according to the log level 0 = SILENT, 1 = INFO, 2 = WARNING, 3 = ERROR, 4 = TRACE, 5 = DEBUG