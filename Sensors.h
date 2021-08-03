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
#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"
#include "IODevice.h"

// Uncomment the following #define statement to use callback notification
//  where the driver supports it.
//  The principle of callback notification is to avoid the Sensor class
//  having to poll the device driver cyclically for input values, and then scan 
//  for changes.  Instead, when the driver scans the inputs, if it detects
//  a change it invokes a callback function in the Sensor class.  In the current
//  implementation, the advantages are limited because (a) the Sensor class 
//  performs debounce checks, and (b) the Sensor class does not have a 
//  static reference to the output stream for sending <Q>/<q> messages
//  when a change is detected.  These restrictions mean that the checkAll() 
//  method still has to iterate through all of the Sensor objects looking 
//  for changes.
#define USE_NOTIFY

struct SensorData {
  int snum;
  VPIN pin;
  uint8_t pullUp;
};

class Sensor{
  // The sensor list is a linked list where each sensor's 'nextSensor' field points to the next.
  //   The pointer is null in the last on the list.
  //   To partition the sensor into those sensors which require polling through cyclic calls
  //   to 'IODevice::read(vpin)', and those which support callback on change, 'firstSensor' 
  //   points to the start of the overall list, and 'lastSensor' points to the end of the list
  //   (the last sensor object). This structure allows sensors to be added to the start or the
  //   end of the list easily.  So if an input pin supports change notification, it is placed at the 
  //   end of the list.  If not, it is placed at the beginning.  And the pointer 'firstPollSensor' 
  //   is set to the first of the sensor objects that requires scanning.  Thus, we can iterate
  //   through the whole list, or just through the part that requires scanning.

public:
  SensorData data;
  struct {
    uint8_t active:1;
    uint8_t inputState:1;
    uint8_t latchDelay:6;
  };   // bit 7=active; bit 6=input state; bits 5-0=latchDelay

  static Sensor *firstSensor;
#ifdef USE_NOTIFY
  static Sensor *firstPollSensor;
  static Sensor *lastSensor;
#endif
  // readingSensor points to the next sensor to be polled, or null if the poll cycle is completed for
  // the period.
  static Sensor *readingSensor;

  // Constructor
  Sensor(); 
  Sensor *nextSensor;
  void setState(int state);
  static void load();
  static void store();
  static Sensor *create(int id, VPIN vpin, int pullUp);
  static Sensor* get(int id);  
  static bool remove(int id);  
  static void checkAll(Print *stream);
  static void printAll(Print *stream);
  static unsigned long lastReadCycle; // value of micros at start of last read cycle
  static const unsigned int cycleInterval = 10000; // min time between consecutive reads of each sensor in microsecs.
                                                   // should not be less than device scan cycle time.
  static const unsigned int minReadCount = 1; // number of additional scans before acting on change
                                        // E.g. 1 means that a change is ignored for one scan and actioned on the next.
                                        // Max value is 63

#ifdef USE_NOTIFY
  static bool pollSignalPhase;
  static void inputChangeCallback(VPIN vpin, int state);
  static IONotifyStateChangeCallback *nextInputChangeCallback;
  static bool inputChangeCallbackRegistered;
#endif
  
}; // Sensor

#endif
