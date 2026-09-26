/*
 *  © 2026 Chris Harlow
 *  
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

#include "Signals.h"
#include "NodeManager.h"
#include "DCC.h"
#include "EXRAIL2.h"
#include "CommandDistributor.h"

Signal *Signal::firstSignal = nullptr;
Signal::Signal(uint16_t _id, const char * _description) {
    id=_id;
    nextSignal=firstSignal;
    firstSignal = this;
    state=SIGNAL_UNKNOWN;
    description=nullptr;
    setRamDescription(_description);
}

Signal::RAG Signal::getState(uint16_t id) {
    return findSignal(id)->state;
}

char * Signal::getDescription(uint16_t id) {
    return findSignal(id)->description;
}

void Signal::action() {
    // Base signal has no hardware action.
}

void Signal::setAllSignalsToRed() {
    for (auto s = firstSignal; s; s = s->nextSignal) {
        s->state = SIGNAL_RED;
        s->action();
        CommandDistributor::broadcastSignal(s->id, s->state, s->getAspect());
    }
}

void Signal::setSignal(uint16_t id, RAG rag, bool tellNodes) {
    if (tellNodes) NodeManager::cast(F("<S %d %c>"), id, rag);
    auto s=findSignal(id);
    bool changed=(s->state!=rag);
    s->state=rag;
    s->action(); // do the hardware implemnentation
    RMFT2::doSignalHandlers(id, rag); // call the ON* handlers
if (changed) CommandDistributor::broadcastSignal(id, rag, s->getAspect()); // broadcast the change to clients
}


bool Signal::setSignalByReverseAspectLookup(uint16_t dccaddress, byte aspect) {
    for (auto s = firstSignal; s; s = s->nextSignal) {
        if (s->ssbral(dccaddress, aspect)) {
            return true;
        }
    }
    return false;
}

bool Signal::ssbral(uint16_t dccaddress, byte aspect) {
    // Default implementation returns false; override in derived classes
    (void)dccaddress; (void)aspect; // avoid unused parameter warning
    return false;
}
void Signal::setRamDescription(const char *desc) {
    if (description) return; // No renaming of signals.
    if (!desc) return; // null description is ignored
    description = (char *)malloc(strlen(desc)+1);
    strcpy(description, desc);
}

bool Signal::isHidden() {
    return description && description[0]==0x01;
}

Signal * Signal::findSignal(uint16_t id) {
    for (auto s=firstSignal;s;s=s->nextSignal) {
        if (s->id == id) return s;
    }
    return new Signal(id, ""); // Create a new signal if not found
}

void Signal::display(Print * stream) {
    for (auto s=firstSignal;s;s=s->nextSignal) {
        switch(s->state) {
            case SIGNAL_RED:    StringFormatter::send(stream,F("Signal %d: RED   %s\n"),s->id,s->description); break;
            case SIGNAL_AMBER:  StringFormatter::send(stream,F("Signal %d: AMBER %s\n"),s->id,s->description); break;
            case SIGNAL_GREEN:  StringFormatter::send(stream,F("Signal %d: GREEN %s\n"),s->id,s->description); break;
            default: StringFormatter::send(stream,F("Signal %d: UNKNOWN %s\n"),s->id,s->description); break;
        }
     }
}
void Signal::listSignalIds(Print * stream){
    for (auto s=firstSignal;s;s=s->nextSignal) {
        if (s->isHidden()) continue;
        StringFormatter::send(stream,F(" %d"),s->id);
    }
}

/* static */ void Signal::shareNodesToCS() {
    
    DIAG(F("Sharing signals to CS"));
    for (auto s=firstSignal;s;s=s->nextSignal) {
      if (s->isHidden()) continue;
	  // todo prevent loopback
      auto description=s->getDescription();
      NodeManager::cast(F("<S %d %d \"%s\">"),s->id, s->state,
       description?description:"");  
    }
  }

// DCC Signals 
DCCSignal::DCCSignal(uint16_t id, uint16_t dccAddress, const char * description) : Signal(id, description), dccAddress(dccAddress) {
}

void DCCSignal::action(){
    DCC::setAccessory((dccAddress - 1) / 4 + 1,(dccAddress - 1)  % 4 ,
         (state!=SIGNAL_RED) ^ accessoryCommandReverse);

}

DCCXSignal::DCCXSignal(uint16_t id, uint16_t dccAddress, byte redAspect, byte amberAspect, byte greenAspect, const char * description) 
    : Signal(id, description), dccAddress(dccAddress), redAspect(redAspect), amberAspect(amberAspect), greenAspect(greenAspect) {
}

void DCCXSignal::action(){
    // Implement DCCX signal setting logic here
    auto value=redAspect;
    switch(state) {
        case SIGNAL_RED:
            value= redAspect;
            break;
        case SIGNAL_AMBER:
            value= amberAspect;
            break;
        case SIGNAL_GREEN:  
            value= greenAspect;
            break;
        default:
            value= redAspect; // default to red if unknown
    }
    currentAspect=value;
    DCC::setExtendedAccessory(dccAddress, value);
}

bool DCCXSignal::ssbral(uint16_t dccaddress, byte aspect) {
    // used when a DCCX signal aspect is altered by a DCC command. 
    // If the dccaddress matches this signal then and the aspect matches
    // one of the defined aspects, then set the state. 
    // DIAG(F("DCCXSignal::ssbral %d %d"), dccaddress, aspect);
    // If this is not the signal in question return false.
    if (dccaddress != dccAddress) return false;
    
    if (currentAspect==aspect) return true; // no change needed 

    if (aspect == redAspect) setSignal(dccaddress, SIGNAL_RED);
    else if (aspect == amberAspect) setSignal(dccaddress, SIGNAL_AMBER);
    else if (aspect == greenAspect) setSignal(dccaddress, SIGNAL_GREEN);
    else {
        currentAspect=aspect; // unknown aspect, just store it
        DCC::setExtendedAccessory(dccAddress, aspect); // set direct..
        // this will recurse but will not change the state since currentAspect is now set to aspect
        CommandDistributor::broadcastSignal(id, state, aspect); // broadcast the change
    }
    return true;
}

// LED signals
LEDSignal::LEDSignal(uint16_t id, VPIN red, VPIN amber, VPIN green, bool invert, const char * description) 
    : Signal(id, description), redPin(red), amberPin(amber), greenPin(green), invert(invert) {
}

void LEDSignal::action(){
    // Implement LED signal setting logic here
    bool simamber=(state==SIGNAL_AMBER && (amberPin==0));
       
    // set the three pins 
    if (redPin) {
        bool redval=(state==SIGNAL_RED || simamber) ^ invert;
        RMFT2::killBlinkOnVpin(redPin);
        IODevice::write(redPin,redval);
    }
    if (amberPin) {
        bool amberval=(state==SIGNAL_AMBER) ^ invert;
        RMFT2::killBlinkOnVpin(amberPin);
        IODevice::write(amberPin,amberval);
    }
    if (greenPin) {
        bool greenval=(state==SIGNAL_GREEN || simamber);
        RMFT2::killBlinkOnVpin(greenPin);
        IODevice::write(greenPin,greenval);
    }
}

NeoPixelSignal::NeoPixelSignal(uint16_t id, VPIN dataPin,  
        uint32_t redRGB, uint32_t amberRGB, uint32_t greenRGB, const char * description) 
    : Signal(id, description), dataPin(dataPin), redRGB(redRGB), amberRGB(amberRGB), greenRGB(greenRGB) {
} 

void NeoPixelSignal::action(){
    // Implement NeoPixel signal setting logic here
    uint32_t colour=redRGB;
    switch(state) {
        case SIGNAL_RED:
            colour= redRGB;
            break;
        case SIGNAL_AMBER:
            colour= amberRGB;
            break;
        case SIGNAL_GREEN:  
            colour= greenRGB;
            break;
        default:
            colour= redRGB; // default to red if unknown
    }
     IODevice::writeAnalogue(id, colour>>8,true,colour & 0xFF); // Assuming colour is in 0xRRGGBB format    
}

ServoSignal::ServoSignal(uint16_t id, VPIN servoPin,  
        uint16_t redAngle, uint16_t amberAngle, uint16_t greenAngle, const char * description) 
    : Signal(id, description), servoPin(servoPin), redAngle(redAngle), amberAngle(amberAngle), greenAngle(greenAngle) {
}

void ServoSignal::action(){

    // Implement Servo signal setting logic here
    uint16_t angle=redAngle;
    switch(state) {
        case SIGNAL_RED:
            angle= redAngle;
            break;
        case SIGNAL_AMBER:
            angle= amberAngle;
            break;
        case SIGNAL_GREEN:  
            angle= greenAngle;
            break;
        default:
            angle= redAngle; // default to red if unknown
    }
     IODevice::writeAnalogue(servoPin, angle,0,0,false);    
}
