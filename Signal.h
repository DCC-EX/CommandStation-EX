#ifndef SIGNAL_H
#define SIGNAL_H
#include "IODevice.h"

class Signal {

    public:
     void setSignal(VPIN id, char rag, bool nodeCast=true);
     char getState(VPIN id);

     private:
     int16_t id;
     char state;
     Signal *nextSignal;
     static Signal *firstSignal;
};

#endif // SIGNAL_H