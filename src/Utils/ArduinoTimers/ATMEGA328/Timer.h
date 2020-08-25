#ifndef ATMEGA328Timer_h
#define ATMEGA328Timer_h

#include "../VirtualTimer.h"
#include <Arduino.h>

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
        //case 0:
        case 2:
            timer_resolution = 256;
            break;
        case 1:
            timer_resolution = 65536;
            break;
        }
        this->timer_num = timer_num;
        lastMicroseconds = 0;
    }

    void initialize() {
        switch (timer_num)
        {
        // case 0:
        //     TCCR0B = _BV(WGM02);
        //     TCCR0A = _BV(WGM00) | _BV(WGM01);
        //     break;
        case 1:
            TCCR1B = _BV(WGM13) | _BV(WGM12);
            TCCR1A = _BV(WGM11);
            break;
        case 2:
            TCCR2B = _BV(WGM22);
            TCCR2A = _BV(WGM20) | _BV(WGM21);
            break;
        }
    }

    void setPeriod(unsigned long microseconds) {
        if(microseconds == lastMicroseconds)
            return;
        lastMicroseconds = microseconds;
        const unsigned long cycles = (F_CPU / 1000000) * microseconds;
        
        switch(timer_num) {
        case 2:
            if (cycles < timer_resolution) {
                clockSelectBits = 1 << 0;
                pwmPeriod = cycles;
            } else
            if (cycles < timer_resolution * 8) {
                clockSelectBits = 1 << 1;
                pwmPeriod = cycles / 8;
            } else
            if (cycles < timer_resolution * 32) {
                clockSelectBits = 1 << 0 | 1 << 1;
                pwmPeriod = cycles / 32;
            } else
            if (cycles < timer_resolution * 64) {
                clockSelectBits = 1 << 2;
                pwmPeriod = cycles / 64;
            } else
            if (cycles < timer_resolution * 128) {
                clockSelectBits = 1 << 2 | 1 << 0;
                pwmPeriod = cycles / 128;
            } else
            if (cycles < timer_resolution * 256) {
                clockSelectBits = 1 << 2 | 1 << 1;
                pwmPeriod = cycles / 256;
            } else
            if (cycles < timer_resolution * 1024) {
                clockSelectBits = 1 << 2 | 1 << 1 | 1 << 0;
                pwmPeriod = cycles / 1024;
            } else {
                clockSelectBits = 1 << 2 | 1 << 1 | 1 << 0;
                pwmPeriod = timer_resolution - 1;
            }
            break;
        //case 0:
        case 1:
            if (cycles < timer_resolution) {
                clockSelectBits = 1 << 0;
                pwmPeriod = cycles;
            } else
            if (cycles < timer_resolution * 8) {
                clockSelectBits = 1 << 1;
                pwmPeriod = cycles / 8;
            } else
            if (cycles < timer_resolution * 64) {
                clockSelectBits = (1 << 0) | (1 << 1);
                pwmPeriod = cycles / 64;
            } else
            if (cycles < timer_resolution * 256) {
                clockSelectBits = 1 << 2;
                pwmPeriod = cycles / 256;
            } else
            if (cycles < timer_resolution * 1024) {
                clockSelectBits = (1 << 2) | (1 << 0);
                pwmPeriod = cycles / 1024;
            } else {
                clockSelectBits = (1 << 2) | (1 << 0);
                pwmPeriod = timer_resolution - 1;
            }
            break;
        }

        switch (timer_num)
        {
        // case 0:
        //     OCR0A = pwmPeriod;
        //     TCCR0B = _BV(WGM02) | clockSelectBits;
        //     break;
        case 1:
            ICR1 = pwmPeriod;
            TCCR1B = _BV(WGM13) | _BV(WGM12) | clockSelectBits;
            break;
        case 2:
            OCR2A = pwmPeriod;
            TCCR2B = _BV(WGM22) | clockSelectBits;
            break;
        }
        
    }
    void start() {
        switch (timer_num)
        {
        // case 0:
        //     TCCR0B = 0;
        //     TCNT0 = 0;		// TODO: does this cause an undesired interrupt?
        //     TCCR0B = _BV(WGM02) | clockSelectBits;
        //     break;
        case 1:
            TCCR1B = 0;
            TCNT1 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR1B = _BV(WGM13) | _BV(WGM12) | clockSelectBits;
            break;
        case 2:
            TCCR2B = 0;
            TCNT2 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR2B = _BV(WGM22) | clockSelectBits;
            break;
        }
        
        
    }
    void stop() {
        switch (timer_num)
        {
        // case 0:
        //     TCCR0B = _BV(WGM02);
        //     break;
        case 1:
            TCCR1B = _BV(WGM13) | _BV(WGM12);
            break;
        case 2:
            TCCR2B = _BV(WGM22);
            break;
        }        
    }

    void attachInterrupt(void (*isr)()) {
        isrCallback = isr;
	    
        switch (timer_num)
        {
        // case 0:
        //     TIMSK0 = _BV(TOIE0); 
        //     break;
        case 1:
            TIMSK1 = _BV(TOIE1); 
            break;
        case 2:
            TIMSK2 = _BV(TOIE2); 
            break;
        }   
    }
    
    void detachInterrupt() {
        switch (timer_num)
        {
        // case 0:
        //     TIMSK0 = 0; 
        //     break;
        case 1:
            TIMSK1 = 0; 
            break;
        case 2:
            TIMSK2 = 0; 
            break;
        }  
    }

};

extern Timer TimerA;
extern Timer TimerB;

#endif