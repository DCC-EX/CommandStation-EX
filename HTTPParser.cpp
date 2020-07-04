/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#include "HTTPParser.h"
#include "StringFormatter.h"

void HTTPParser::parse(Print & stream, byte * cmd) {
     (void)cmd;  // Avoid compiler warning because this example doesnt use this parameter
      
     // BEWARE   - As soon as you start responding, the cmd buffer is trashed!
     // You must get everything you need from it before using StringFormatter::send!
       
     StringFormatter::send(stream,F("HTTP/1.1 200 OK\nContent-Type: text/html\nConnnection: close\n\n"));
     StringFormatter::send(stream,F("<html><body>This is <b>not</b> a web server.<br/></body></html>"));    
}
