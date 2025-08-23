#include "SensorGroup.h"
#include "CommandDistributor.h"

#ifdef EXRAIL_ACTIVE

// called in loop to check sensors
void SensorGroup::checkAll() {
    doExrailSensorGroup(GroupProcess::check, & USB_SERIAL);
}

// called by command to get sensor list   
void SensorGroup::printAll(Print * serial) {
    (void)serial; // suppress unused warning
    doExrailSensorGroup(GroupProcess::print,serial);
}

void SensorGroup::prepareAll() {
    doExrailSensorGroup(GroupProcess::prepare, & USB_SERIAL);
}

void SensorGroup::dumpAll(Print * stream) {
    doExrailSensorGroup(GroupProcess::dump, stream);
}

#else
// if EXRAIL is not active, these functions are empty
void SensorGroup::checkAll() {}
void SensorGroup::printAll(Print * serial) {(void)serial;}
void SensorGroup::prepareAll() {}
void SensorGroup::dumpAll(Print * stream) {(void)stream;}
#endif 

// called by EXRAIL constructed doExrailSensorGroup for each group 
void SensorGroup::doSensorGroup(VPIN firstVpin, int nPins, byte* statebits,
  GroupProcess action, Print * serial, bool pullup) {
  
  // Loop through the pins in the group  
  for (auto i=0;i<nPins;i++) {   
    // locate position of state bit
    byte stateByte=i/8;
    byte stateMask=1<<(i%8);
    VPIN vpin= firstVpin+i;
    switch(action) {
      case GroupProcess::prepare:
          IODevice::configureInput(vpin, pullup);
          if (IODevice::read(vpin))  statebits[stateByte]|=stateMask;
          break; 
    
      case GroupProcess::check:
         // check for state unchanged
         if ((bool)(statebits[stateByte]&stateMask) == IODevice::read(vpin)) break; // no change  
         // flip state bit
         statebits[stateByte]^=stateMask;
         CommandDistributor::broadcastSensor(vpin,statebits[stateByte]&stateMask);
         break;
      
      case GroupProcess::print:
        StringFormatter::send(serial, F("<%c %d>\n"), 
         (statebits[stateByte]&stateMask)?'Q':'q', vpin);
         break;

      case GroupProcess::dump:
        StringFormatter::send(serial, F("<Q %d %d %c>\n"), 
         vpin, vpin, pullup?'1':'0');
         break;
    } 
  }
}
    