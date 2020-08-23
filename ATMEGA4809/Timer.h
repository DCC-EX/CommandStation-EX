#ifndef ATMEGA328Timer_h
#define ATMEGA328Timer_h

#include "../VirtualTimer.h"
#include <Arduino.h>

// We only define behavior for timer 0 (TCA0), because TCB0 is very limited in functionality.

class Timer : public VirtualTimer {
private:
    int pwmPeriod;
    unsigned long timer_resolution;
    unsigned char clockSelectBits;
    int timer_num;
    unsigned long lastMicroseconds;
public:
    void (*isrCallback)();
    Timer(int timer_num) {
        switch (timer_num)
        {
        case 0:
            timer_resolution = 65536;
            break;
        }
        this->timer_num = timer_num;
        lastMicroseconds = 0;
    }

    void initialize() {
        switch (timer_num)
        {
        case 0:
            break;
        }
    }

    void setPeriod(unsigned long microseconds) {
        if(microseconds == lastMicroseconds)
            return;
        lastMicroseconds = microseconds;
        const unsigned long cycles = (F_CPU / 1000000) * microseconds;
        
        switch(timer_num) {
        case 0:
            if (cycles < timer_resolution) {
                clockSelectBits = 0x0;
                pwmPeriod = cycles;
            } else
            if (cycles < timer_resolution * 2) {
                clockSelectBits = 0x1;
                pwmPeriod = cycles / 8;
            } else
            if (cycles < timer_resolution * 4) {
                clockSelectBits = 0x2;
                pwmPeriod = cycles / 32;
            } else
            if (cycles < timer_resolution * 8) {
                clockSelectBits = 0x3;
                pwmPeriod = cycles / 64;
            } else
            if (cycles < timer_resolution * 64) {
                clockSelectBits = 0x5;
                pwmPeriod = cycles / 128;
            } else
            if (cycles < timer_resolution * 256) {
                clockSelectBits = 0x6;
                pwmPeriod = cycles / 256;
            } else
            if (cycles < timer_resolution * 1024) {
                clockSelectBits = 0x7;
                pwmPeriod = cycles / 1024;
            } else {
                clockSelectBits = 0x7;
                pwmPeriod = timer_resolution - 1;
            }
            break;
        }

        switch (timer_num)
        {
        case 0:
            TCA0.SINGLE.PER = pwmPeriod;
            TCA0.SINGLE.CTRLA = clockSelectBits << 1;
            break;   
        }     
    }

    void start() {
        switch (timer_num)
        {
            case 0:
                bitSet(TCA0.SINGLE.CTRLA, 0);
                break;
        }
        
        
    }
    void stop() {
        switch (timer_num)
        {
        case 0:
            bitClear(TCA0.SINGLE.CTRLA, 0);
            break;
        }        
    }

    void attachInterrupt(void (*isr)()) {
        isrCallback = isr;
	    
        switch (timer_num)
        {
        case 0:
            TCA0.SINGLE.INTCTRL = 0x1; 
            break;
        }   
    }
    
    void detachInterrupt() {
        switch (timer_num)
        {
        case 0:
            TCA0.SINGLE.INTCTRL = 0x0; 
            break;
        }  
    }

};

extern Timer TimerA;

#endif