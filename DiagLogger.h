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

#ifndef DiagLogger_h
#define DiagLogger_h

#include "StringFormatter.h"


#define MAXWRITERS 10
typedef void (*DiagWriter)(const char *msg, const int length);

class DiagLogger
{

private:
    // Methods
    DiagLogger() = default;
    DiagLogger(const DiagLogger &);            // non construction-copyable
    DiagLogger &operator=(const DiagLogger &); // non copyable

    // Members
    static DiagLogger singleton; // unique instance of the MQTTInterface object
    DiagWriter writers[MAXWRITERS];
    int registered = 0; // number of registered writers ( Serial is not counted as always used )
    char b1[256];
    
public:
    // Methods
    static DiagLogger &get() noexcept
    { // return a reference to the unique instance
        return singleton;
    }

    void diag(const FSH *input...);
    void addDiagWriter(DiagWriter l);
    ~DiagLogger() = default;

    // Members
};

#endif