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
     
     static void setSignal(uint16_t id, RAG rag, bool nodeCast=true);
     static RAG getState(uint16_t id);
     static void setAllSignalsToRed();
     static void display(Print * stream);
     static bool setSignalByReverseAspectLookup(uint16_t id, byte aspect);
     Signal(uint16_t id);
    
    private:
     Signal *nextSignal;
     virtual void action();
     virtual bool ssbral(uint16_t id, byte aspect);
     static Signal *firstSignal;
     static Signal* findSignal(uint16_t id);

    protected:
        uint16_t id;
        RAG state;
#ifdef DCC_ACCESSORY_COMMAND_REVERSE

    static const bool accessoryCommandReverse = true;
#else    
    static const bool accessoryCommandReverse = false;
#endif
};

class DCCSignal : public Signal {
    public:
     DCCSignal(uint16_t id, uint16_t dccAddress);
    private:
     uint16_t dccAddress;
     void action() override; 
};

class DCCXSignal: public Signal {
    public:
     DCCXSignal(uint16_t id, uint16_t dccAddress, byte redAspect, byte amberAspect, byte greenAspect);
     bool ssbral(uint16_t dccaddress, byte aspect) override;
     private:
     uint16_t dccAddress;
     byte redAspect,amberAspect,greenAspect;

     void action() override; 
};

class LEDSignal : public Signal {
    public:
     LEDSignal(uint16_t id, VPIN red, VPIN amber, VPIN green, bool invert=false);
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
        uint32_t redRGB, uint32_t amberRGB, uint32_t greenRGB);
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
        uint16_t redAngle, uint16_t amberAngle, uint16_t greenAngle);
    private:
      VPIN servoPin;
      uint16_t redAngle;
      uint16_t amberAngle;
      uint16_t greenAngle;
      void action() override;
};
#endif // SIGNAL_H