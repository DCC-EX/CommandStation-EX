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
#ifndef SIGNAL_H
#define SIGNAL_H
#include "IODevice.h"

class Signal {

    public:
     enum RAG:char {
        SIGNAL_RED='R',
        SIGNAL_AMBER='A',
        SIGNAL_GREEN='G',
        SIGNAL_SIMAMBER='S',
        SIGNAL_UNKNOWN='X'
    }; // S is a special case for simulatd amber when no amber pin is defined. It is treated as amber for the purposes of ON* handlers but as red for the purposes of setting the hardware.
     
     static void setSignal(uint16_t id, RAG rag, bool tellNodes=true);
     static RAG getState(uint16_t id);
     static char * getDescription(uint16_t id);
     static void setAllSignalsToRed();
     static void display(Print * stream);
     static bool setSignalByReverseAspectLookup(uint16_t id, byte aspect);
     static void shareNodesToCS();
     static void listSignalIds(Print * stream);
     static Signal* findSignal(uint16_t id);

     Signal(uint16_t id, const char * description);
     uint16_t getId() { return id; }
     RAG getState() { return state; }
     const char * getDescription() { return description; }
     void setRamDescription(const char *desc);
     bool isHidden();
    private:
     Signal *nextSignal;
     virtual void action();
     virtual bool ssbral(uint16_t id, byte aspect); // set signal by reverse aspect lookup
     static Signal *firstSignal;
     
    protected:
        uint16_t id;
        RAG state;
        char * description;
#ifdef DCC_ACCESSORY_COMMAND_REVERSE

    static const bool accessoryCommandReverse = true;
#else    
    static const bool accessoryCommandReverse = false;
#endif
};

class DCCSignal : public Signal {
    public:
     DCCSignal(uint16_t id, uint16_t dccAddress, const char * description);
    private:
     uint16_t dccAddress;
     void action() override; 
};

class DCCXSignal: public Signal {
    public:
     DCCXSignal(uint16_t id, uint16_t dccAddress, byte redAspect, byte amberAspect, byte greenAspect, const char * description);
     bool ssbral(uint16_t dccaddress, byte aspect) override;
     private:
     uint16_t dccAddress;
     byte redAspect,amberAspect,greenAspect;

     void action() override; 
};

class LEDSignal : public Signal {
    public:
    LEDSignal(uint16_t id, VPIN red, VPIN amber, VPIN green, bool invert=false, const char * description=nullptr);
    private:
     VPIN redPin;
     VPIN amberPin;
     VPIN greenPin;
     bool invert;
     void action() override;
};

class NeoPixelSignal : public Signal {
    public:
     NeoPixelSignal(uint16_t id, VPIN dataPin,  
        uint32_t redRGB, uint32_t amberRGB, uint32_t greenRGB, const char * description);
    private:
      VPIN dataPin;
      uint32_t redRGB;
      uint32_t amberRGB;
      uint32_t greenRGB;
      void action() override;
};

class ServoSignal : public Signal {
    public:
     ServoSignal(uint16_t id, VPIN servoPin,  
        uint16_t redAngle, uint16_t amberAngle, uint16_t greenAngle, const char * description);
    private:
      VPIN servoPin;
      uint16_t redAngle;
      uint16_t amberAngle;
      uint16_t greenAngle;
      void action() override;
};
#endif // SIGNAL_H