/*
 *  © 2026 Chris Harlow
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

 #ifndef ROSTER_H
#define ROSTER_H
#include <Arduino.h>

class Roster {
public:
    // Create from prog code 
    static void create(int locoId, const char* name, const char* functionMap= nullptr);
    static void remove(int locoId);
    static void sendIdsToThrottle(Print * stream);
    static void sendToThrottle(int locoId, Print * stream);
    static void sendToWithrottle(Print * stream);
    static void sendAsJavascript(Print * stream);
    static const char * getFunctionNamesForWithrottle(int locoid);
    static void save();
    static void load();
private:
    static Roster * find(int locoId);
    Roster();
    void sendToThrottle(Print * stream,bool asSave=false);
    static Roster * first;
    Roster * next;
    int locoId;
    char* name;
    char* functionMap;
};

#endif // ROSTER_H  