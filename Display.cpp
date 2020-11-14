/*
 *  © 2020, Harald Barth.
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
#include <Arduino.h>
#include "Display.h"
#include "DIAG.h"

void Display::displayIP(byte ip[4]) {
    if (ip != NULL)
	LCD(6,F("%d.%d.%d.%d"),ip[0],ip[1],ip[2],ip[3]);
    else
	LCD(6,F("NO IP"));
}
void Display::displayIP(char *ip) {
    if (ip != NULL)
	LCD(6,F("%s"),ip);
    else
	LCD(6,F("NO IP"));
}
void Display::displayPort(int i) {
    if (i > 0)
	LCD(7,F(":%d"),i);
    else
	LCD(7,F("none"));
}
