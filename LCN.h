/*
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
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
#ifndef LCN_h
#define LCN_h
#include <Arduino.h>

class LCN {
  public: 
    static void init(Stream & lcnstream);
    static void loop();
    static void send(char opcode, int id, bool state);
  private :
    static bool firstLoop; 
    static Stream * stream; 
    static int id;
};

#endif
