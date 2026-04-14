#ifndef SensorGroup_h
#define SensorGroup_h
#include <Arduino.h>
#include "defines.h"
#include "IODevice.h"
#include "StringFormatter.h"


// reference to the optional exrail built function which contains the 
// calls to SensorGroup::doSensorGroup 

enum GroupProcess:byte {prepare,print,check,dump};

class SensorGroup {
    public:
     static void checkAll();
     static void printAll(Print * serial);
     static void prepareAll();
     static void dumpAll(Print* serial);

     // doSensorGroup is called from the automatically 
     // built doExrailSensorGroup, once for each user defined group.
     static void doSensorGroup(VPIN vpin, int nPins, byte* statebits,
        GroupProcess action, Print * serial, bool pullup=false);
     private: 
       static void doExrailSensorGroup(GroupProcess action, Print * stream);  
};
#endif // SensorGroup_h
