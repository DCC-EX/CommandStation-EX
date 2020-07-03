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
#ifndef Hardware_h
#define Hardware_h
// Virtualised hardware Interface
class Hardware {
  public:
    static void init();
    static void setPower(bool isMainTrack, bool on);
    static void setSignal(bool isMainTrack, bool high);
    static unsigned int  getCurrentMilliamps(bool isMainTrack, int rawValue);
    static int  getCurrentRaw(bool isMainTrack);
    static void setBrake(bool isMainTrack, bool on);
    static void setCallback(int duration,  void (*isr)());
//    static void setSingleCallback(int duration,  void (*isr)());
//    static void resetSingleCallback(int duration);
};
#endif
