/*
 *  © 2024 Chris Harlow
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

/**********************************************************************
EXRAILSensor represents a sensor that should be monitored in order
to call an exrail ONBUTTON or ONCHANGE handler.
These are created at EXRAIL startup and thus need no delete or listing
capability.
The basic logic is similar to that found in the Sensor class
except that on the relevant change an EXRAIL thread is started.    
**********************************************************************/

#include "EXRAILSensor.h"
#include "EXRAIL2.h"

void EXRAILSensor::checkAll() {
  if (firstSensor == NULL) return;  // No sensors to be scanned
  if (readingSensor == NULL) { 
    // Not currently scanning sensor list
    unsigned long thisTime = micros();
    if (thisTime - lastReadCycle < cycleInterval) return;
    // Required time has elapsed since last read cycle started,
    // so initiate new scan through the sensor list
    readingSensor = firstSensor;
    lastReadCycle = thisTime;
  }
  
  // Loop until either end of list is encountered or we pause for some reason
  byte sensorCount = 0;

  while (readingSensor != NULL) {
    bool pause=readingSensor->check();
    // Move to next sensor in list.
    readingSensor = readingSensor->nextSensor;
    // Currently process max of 16 sensors per entry.
    // Performance measurements taken during development indicate that, with 128 sensors configured
    // on 8x 16-pin MCP23017 GPIO expanders with polling (no change notification), all inputs can be read from the devices
    // within 1.4ms (400Mhz I2C bus speed), and a full cycle of checking 128 sensors for changes takes under a millisecond.
    if (pause || (++sensorCount)>=16) return; 
  }
} 

bool EXRAILSensor::check() {
  // check for debounced change in this sensor 
  inputState = RMFT2::readSensor(pin);

  // Check if changed since last time, and process changes.
  if (inputState == active) {// no change
    latchDelay = minReadCount; // Reset counter
    return false;  // no change 
  }

    // Change detected ... has it stayed changed for long enough
    if (latchDelay > 0) {
      latchDelay--;
      return false; 
    } 
    
    // change validated, act on it.
    active = inputState;
    latchDelay = minReadCount;  // Reset debounce counter
    if (onChange || active) {
      new RMFT2(progCounter);
      return true;  // Don't check any more sensors on this entry
    }
    return false; 
}

EXRAILSensor::EXRAILSensor(VPIN _pin, int _progCounter, bool _onChange) {
  // Add to the start of the list
  //DIAG(F("ONthing vpin=%d at %d"), _pin, _progCounter);
  nextSensor = firstSensor;
  firstSensor = this;

  pin=_pin;
  progCounter=_progCounter;
  onChange=_onChange;

  IODevice::configureInput(pin, true);   
  active = IODevice::read(pin);
  inputState = active;
  latchDelay = minReadCount;
}

EXRAILSensor *EXRAILSensor::firstSensor=NULL;
EXRAILSensor *EXRAILSensor::readingSensor=NULL;
unsigned long EXRAILSensor::lastReadCycle=0;
