#include "Signals.h"
#include "NodeManager.h"
#include "DCC.h"
#include "EXRAIL2.h"

Signal *Signal::firstSignal = nullptr;
Signal::Signal(uint16_t _id) {
    id=_id;
    nextSignal=firstSignal;
    firstSignal = this;
    state=SIGNAL_UNKNOWN;
}

Signal::RAG Signal::getState(uint16_t id) {
    return findSignal(id)->state;
}

void Signal::action() {
    // Base signal has no hardware action.
}

void Signal::setAllSignalsToRed() {
    for (auto s = firstSignal; s; s = s->nextSignal) {
        s->state = SIGNAL_RED;
        s->action();
    }
}

void Signal::setSignal(uint16_t id, RAG rag, bool nodeCast) {
    if (nodeCast) NodeManager::cast(F("<S %d %c>"), id, rag);
    auto s=findSignal(id);
    s->state=rag;
    s->action(); // do the hardware implemnentation
    RMFT2::doSignalHandlers(id, rag); // call the ON* handlers 
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

Signal * Signal::findSignal(uint16_t id) {
    for (auto s=firstSignal;s;s=s->nextSignal) {
        if (s->id == id) return s;
    }
    return new Signal(id); // Create a new signal if not found
}

void Signal::display(Print * stream) {
    for (auto s=firstSignal;s;s=s->nextSignal) {
        switch(s->state) {
            case SIGNAL_RED:  StringFormatter::send(stream,F("Signal %d: RED\n"),s->id); break;
            case SIGNAL_AMBER:StringFormatter::send(stream,F("Signal %d: AMBER\n"),s->id); break;
            case SIGNAL_GREEN: StringFormatter::send(stream,F("Signal %d: GREEN\n"),s->id); break;
            default: StringFormatter::send(stream,F("Signal %d: UNKNOWN\n"),s->id); break;
        }
     }
}
// DCC Signals 
DCCSignal::DCCSignal(uint16_t id, uint16_t dccAddress) : Signal(id), dccAddress(dccAddress) {
}

void DCCSignal::action(){
    DCC::setAccessory((dccAddress - 1) / 4 + 1,(dccAddress - 1)  % 4 ,
         (state!=SIGNAL_RED) ^ accessoryCommandReverse);

}

DCCXSignal::DCCXSignal(uint16_t id, uint16_t dccAddress, byte redAspect, byte amberAspect, byte greenAspect) 
    : Signal(id), dccAddress(dccAddress), redAspect(redAspect), amberAspect(amberAspect), greenAspect(greenAspect) {
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
    DCC::setExtendedAccessory(dccAddress, value);
}

bool DCCXSignal::ssbral(uint16_t dccaddress, byte aspect) {
    // used when a DCCX signal aspect is altered by a DCC command. 
    // If the dccaddress matches this signal and the aspect matches
    // one of the defined aspects, then set the state and return true. 
    // Otherwise return false.
    if (dccaddress != dccAddress) return false;

    if (aspect == redAspect) state=SIGNAL_RED;
    else if (aspect == amberAspect) state=SIGNAL_AMBER;
    else if (aspect == greenAspect) state=SIGNAL_GREEN;
    else {
        DCC::setExtendedAccessory(dccAddress, aspect); // set to red if unknown aspect
        return true; // return true to indicate that the aspect was handled, even if the aspect was unknown
    }
    setSignal(id, state); // update the hardware/nodes
    return true;
}

// LED signals
LEDSignal::LEDSignal(uint16_t id, VPIN red, VPIN amber, VPIN green, bool invert) 
    : Signal(id), redPin(red), amberPin(amber), greenPin(green), invert(invert) {
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
        uint32_t redRGB, uint32_t amberRGB, uint32_t greenRGB) 
    : Signal(id), dataPin(dataPin), redRGB(redRGB), amberRGB(amberRGB), greenRGB(greenRGB) {
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
        uint16_t redAngle, uint16_t amberAngle, uint16_t greenAngle) 
    : Signal(id), servoPin(servoPin), redAngle(redAngle), amberAngle(amberAngle), greenAngle(greenAngle) {
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
     IODevice::writeAnalogue(servoPin, angle);    
}
