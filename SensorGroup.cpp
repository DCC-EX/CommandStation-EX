#include "SensorGroup.h"
#include "CommandDistributor.h"
#include "NodeManager.h"

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

void SensorGroup::shareSensorsToCS() {
    doExrailSensorGroup(GroupProcess::share, nullptr);
}

#else
// if EXRAIL is not active, these functions are empty
void SensorGroup::checkAll() {}
void SensorGroup::printAll(Print * serial) {(void)serial;}
void SensorGroup::prepareAll() {}
void SensorGroup::dumpAll(Print * stream) {(void)stream;}
void SensorGroup::shareSensorsToCS() {}

#endif 

// called by EXRAIL constructed doExrailSensorGroup for each group 
void SensorGroup::doJMRISensorGroup(VPIN firstVpin, int nPins, byte* statebits,
  GroupProcess action, Print * serial, bool pullup) {
         
  // Loop through the pins in the group  
  for (auto i=0;i<nPins;i++) {
    // locate position of state bit
    byte stateByte=i/8;
    byte stateMask=1<<(i%8);
    VPIN vpin= firstVpin+i;
    switch(action) {
      case GroupProcess::prepare:
            // this is monitoring Vpins, so configure them as inputs
            IODevice::configureInput(vpin,pullup);
            if (IODevice::read(vpin))  statebits[stateByte]|=stateMask;
          break; 
    
      case GroupProcess::check:
        {
         // dont check incoming shared groups, they are updated by the other node
         // check for state unchanged
         if ((bool)(statebits[stateByte]&stateMask) == IODevice::read(vpin)) break; // no change  
         // flip state bit
         statebits[stateByte]^=stateMask;
         bool state=statebits[stateByte]&stateMask;
           // if not shared, broadcast to JMRI etc.
           CommandDistributor::broadcastSensor(vpin,state);
        }
        break;
      
      case GroupProcess::print:
        StringFormatter::send(serial, F("<%c %d>\n"), 
         (statebits[stateByte]&stateMask)?'Q':'q', vpin);
         break;

      case GroupProcess::dump:
        StringFormatter::send(serial, F("<Q %d %d %c>\n"), 
         vpin, vpin, pullup?'1':'0');
         break;

      case GroupProcess::share:
         break;
    } 
  }
}

    // called by EXRAIL constructed doExrailSensorGroup for each group 
void SensorGroup::doSharedSensorGroup(VPIN firstVpin, int nPins, byte* statebits,
  GroupProcess action, GroupType * groupType) {

  switch(action) {
    case GroupProcess::prepare:
        //Need to know if this group is monitoring real pins or shared pins from another node.

        if (IODevice::checkNoOverlap(firstVpin,nPins,0,true)){
          // if  no overlap, then this is monitoring pins from another node
          // so we have to create a local copy of the state bits
          DIAG(F("Creating incoming SHARED_SENSOR( %d,%d)"),firstVpin,nPins);
          FLAGS::create(firstVpin,nPins);
          *groupType=GroupType::incomingShare;  // mark this group as shared
          return; 
        }

        // this is monitoring real pins, so configure them as inputs and take current state
        DIAG(F("Creating outgoing SHARED_SENSOR( %d,%d)"),firstVpin,nPins);
        for (auto i=0;i<nPins;i++) {
          byte stateByte=i/8;
          byte stateMask=1<<(i%8);
          VPIN vpin= firstVpin+i;
          IODevice::configureInput(vpin,true);
          if (IODevice::read(vpin))  statebits[stateByte]|=stateMask;
        }
      
        *groupType=GroupType::outgoingShare;  // mark this group as outgoing
        break;
  
    case GroupProcess::check:
        if (*groupType!=GroupType::outgoingShare) return; // dont check incoming shared groups
        // Loop through the pins in the group  
        for (auto i=0;i<nPins;i++) {
          // locate position of state bit
          byte stateByte=i/8;
          byte stateMask=1<<(i%8);
          VPIN vpin= firstVpin+i;
          // check for state unchanged
          if ((bool)(statebits[stateByte]&stateMask) == IODevice::read(vpin)) break; // no change  
          // flip state bit
          statebits[stateByte]^=stateMask;
          bool state=statebits[stateByte]&stateMask;
          NodeManager::cast(F("<q %d %b>\n"),vpin, state);
        } 
        break;

    case GroupProcess::print:
    case GroupProcess::dump:
        DIAG(F("%S SHARED_SENSOR(%d,%d)"),
        *groupType==GroupType::outgoingShare ? F("Outgoing") : F("Incoming"), firstVpin,nPins);
        break;

    case GroupProcess::share: // share outgoing groups to the CS and other nodes 
      if (*groupType!=GroupType::outgoingShare) break; 
      // share the outgoing group to the CS as 8 bits per xmit
      for (auto partbyte=0;partbyte<(nPins+7)/8;partbyte++) {
        auto vpin=firstVpin+8*partbyte;
        auto npins=nPins-8*partbyte;
        if (npins>8) npins=8;
        NodeManager::cast(F("<q %d %d %d>"),vpin, npins, statebits[partbyte]);
      }
      break;
    } // switch(action)
}
