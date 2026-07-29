#ifndef SensorGroup_h
#define SensorGroup_h
#include <Arduino.h>
#include "defines.h"
#include "IODevice.h"
#include "StringFormatter.h"


// reference to the optional exrail built function which contains the 
// calls to SensorGroup::doSensorGroup 

enum GroupProcess:byte {prepare,print,check,dump,share,apply};
enum GroupType:byte {jmri,shared,outgoingShare,incomingShare};

class SensorGroup {
    public:
     static void checkAll();
     static void printAll(Print * serial);
     static void prepareAll();
     static void dumpAll(Print* serial);
     static void shareSensorsToCS();
     static void applyIncomingSensor(VPIN applyPin, bool applyState);
     
     // doSensorGroup is called from the automatically 
     // built doExrailSensorGroup, once for each user defined group.
     static void doJMRISensorGroup(VPIN vpin, int nPins, byte* statebits,
        GroupProcess action, Print * serial, bool pullup);
     static void doSharedSensorGroup(VPIN vpin, int nPins, byte* statebits,
        GroupProcess action, VPIN applyPin=0, bool applyState=false);
     static void doRemoteSensorGroup(VPIN vpin, int nPins,
        GroupProcess action, VPIN applyPin=0, bool applyState=false);
     private: 
       static void doExrailSensorGroup(GroupProcess action, Print * stream, VPIN applyPin=0, bool applyState=false);  
};
#endif // SensorGroup_h
