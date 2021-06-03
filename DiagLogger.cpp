/*
 *  © 2021, Gregor Baues, All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
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
 * 
 */

#include "DiagLogger.h"

// DIAG.h the #define DIAG points to here ... 
// EthernetSetup , Wifisetup, etc can register a function to be called allowing the channel
// to publish the diag info to 
// serial is default end enabled all the time

DiagLogger DiagLogger::singleton;          // static instantiation;

void DiagLogger::addDiagWriter(DiagWriter l) {
    if ( registered == MAXWRITERS ) {
        Serial.println("Error: Max amount of writers exceeded.");
        return;
    }
    writers[registered] = l;
    registered++;
}


void DiagLogger::diag(const FSH *input,...)
{ 

    va_list args;
    va_start(args, input);

    int len = 0;
    len += sprintf(&b1[len], "<* ");
    len += vsprintf_P(&b1[len], (const char *)input, args); 
    len += sprintf(&b1[len], " *>\n");

    if ( len >= 256 ) { Serial.print("ERROR : Diag Buffer overflow"); return; }
    // allways print to Serial 
    Serial.print(b1);

    // callback the other registered diag writers 
    for (size_t i = 0; i < (size_t) registered; i++)
    {
        writers[i](b1, len);   
    }

    va_end(args);

}