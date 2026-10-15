/*
 *  © 2021 Neil McKechnie
 *  © 2020-2021 Harald Barth
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
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
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "IODevice.h"


class Sensor{
  friend class AnalogSensor;
  public: 
    static bool createJMRI(int id, VPIN vpin, bool pullUp, byte count=1);
    static bool createNode(VPIN vpin);
    static bool createEXRAIL(VPIN vpin, bool analogue=false);

    static void removeJMRI(int id);  
    static void loop();
    static void printAllJMRI(Print *stream);
    static void dumpAllJMRI(Print* stream);
    static void shareNodesToCS();
    
  
  // The sensor list is a linked list where each sensor's 'nextSensor' field points to the next.
  //   The pointer is null in the last on the list.

private:
  struct {
    uint8_t pullUp:1;
    uint8_t active:1;
    uint8_t inputState:1;
    uint8_t analogue:1;
    uint8_t tellJMRI:1;
    uint8_t tellNodes:1;
    uint8_t tellEXRAIL:1;
    
  };   
  int id;
  VPIN vpin;
  uint8_t latchDelay;
  Sensor *nextSensor;
  
  
  static Sensor *firstSensor;
  // readingSensor points to the next sensor to be polled, or null if the poll cycle is completed for
  // the period.
  static Sensor *readingSensor;

  Sensor(int id,VPIN vpin, bool pullup); 
  virtual void check();
  static Sensor* get(int id, VPIN vpin);
  static Sensor* getByVpin(VPIN vpin);
  static Sensor* getById(int id);
  
  
     
  static unsigned long lastReadCycle; // value of micros at start of last read cycle
  static const unsigned int cycleInterval = 10000; // min time between consecutive reads of each sensor in microsecs.
                                                   // should not be less than device scan cycle time.
  static const unsigned int minReadCount = 1; // number of additional scans before acting on change
                                        // E.g. 1 means that a change is ignored for one scan and actioned on the next.
                                        // Max value is 63

  
  
  
}; // Sensor
class AnalogSensor : public Sensor {
  friend class Sensor;
  private:
    uint8_t analogActive;
    uint8_t analogInputState;

    AnalogSensor(VPIN vpin) : Sensor(-99, vpin, false) {
      analogue = true;
      analogActive = 0;
    }
    void check();  // override for analog sensor

};
#endif
