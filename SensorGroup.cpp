#include "SensorGroup.h"

// called in loop to check sensors
void SensorGroup::checkAll() {
    #ifdef EXRAIL_ACTIVE
    doExrailSensorGroup(GroupProcess::CHECK, & USB_SERIAL);
    #endif
}

// called by command to get sensor list   
void SensorGroup::printAll(Print * serial) {
    (void)serial; // suppress unused warning
    #ifdef EXRAIL_ACTIVE
    doExrailSensorGroup(GroupProcess::PRINT,serial);
    #endif
}

void SensorGroup::pullupAll() {
    #ifdef EXRAIL_ACTIVE
    doExrailSensorGroup(GroupProcess::PULLUP, & USB_SERIAL);
    #endif
}

// called by EXRAIL constructed doExrailSensorGroup for each group 
void SensorGroup::doSensorGroup(VPIN firstVpin, int nPins, byte* statebits,
  GroupProcess action, Print * serial) {
  
  // Loop through the pins in the group  
  for (auto i=0;i<nPins;i++) {   
    // locate position of state bit
    byte stateByte=i/8;
    byte stateMask=1<<(i%8);
    VPIN vpin= firstVpin+i;
    switch(action) {
      case GroupProcess::PULLUP:
          IODevice::configureInput(vpin, true);
          __attribute__ ((fallthrough)); // to  check the current state 
    
      case GroupProcess::CHECK:
         // check for state change
         if ((bool)(statebits[stateByte]&stateMask) ==IODevice::read(vpin)) break; // no change  
         // flip state bit
         statebits[stateByte]^=stateMask;
         if (action==GroupProcess::PULLUP) break; 
         // fall through to print the changed value  
        __attribute__ ((fallthrough));
      
      case GroupProcess::PRINT:
        StringFormatter::send(serial, F("<%c %d>\n"), 
         (statebits[stateByte]&stateMask)?'Q':'q', firstVpin+i);
         break;
    } 
  }
}
    