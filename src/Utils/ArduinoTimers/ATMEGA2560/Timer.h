#ifndef ATMEGA2560Timer_h
#define ATMEGA2560Timer_h

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
        case 1:
        case 3:
        case 4:
        case 5:
            timer_resolution = 65536;
            break;
        }
        this->timer_num = timer_num;
        lastMicroseconds = 0;
    }

    void initialize() {
        switch (timer_num)
        {
        case 1:
            TCCR1B = _BV(WGM13) | _BV(WGM12);
            TCCR1A = _BV(WGM11);
            break;
        case 3:
            TCCR3B = _BV(WGM33) | _BV(WGM32);
            TCCR3A = _BV(WGM31);
            break;
        case 4:
            TCCR4B = _BV(WGM43) | _BV(WGM42);
            TCCR4A = _BV(WGM41);
            break;
        case 5:
            TCCR5B = _BV(WGM53) | _BV(WGM52);
            TCCR5A = _BV(WGM51);
            break;
        }
    }

    void setPeriod(unsigned long microseconds) {
        if(microseconds == lastMicroseconds)
            return;
        lastMicroseconds = microseconds;
        const unsigned long cycles = (F_CPU / 1000000) * microseconds;
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

        switch (timer_num)
        {
        case 1:
            ICR1 = pwmPeriod;
            TCCR1B = _BV(WGM13) | _BV(WGM12) | clockSelectBits;
            break;
        case 3:
            ICR3 = pwmPeriod;
            TCCR3B = _BV(WGM33) | _BV(WGM32) | clockSelectBits;
            break;
        case 4:
            ICR4 = pwmPeriod;
            TCCR4B = _BV(WGM43) | _BV(WGM42) | clockSelectBits;
            break;
        case 5:
            ICR5 = pwmPeriod;
            TCCR5B = _BV(WGM53) | _BV(WGM52) | clockSelectBits;
            break;
        }
        
    }
    void start() {
        switch (timer_num)
        {
        case 1:
            TCCR1B = 0;
            TCNT1 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR1B = _BV(WGM13) | _BV(WGM12) | clockSelectBits;
            break;
        case 3:
            TCCR3B = 0;
            TCNT3 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR3B = _BV(WGM33) | _BV(WGM32) | clockSelectBits;
            break;
        case 4:
            TCCR4B = 0;
            TCNT4 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR4B = _BV(WGM43) | _BV(WGM42) | clockSelectBits;
            break;
        case 5:
            TCCR5B = 0;
            TCNT5 = 0;		// TODO: does this cause an undesired interrupt?
            TCCR5B = _BV(WGM53) | _BV(WGM52) | clockSelectBits;
            break;
        }
        
        
    }
    void stop() {
        switch (timer_num)
        {
        case 1:
            TCCR1B = _BV(WGM13) | _BV(WGM12);
            break;
        case 3:
            TCCR3B = _BV(WGM33) | _BV(WGM32);
            break;
        case 4:
            TCCR4B = _BV(WGM43) | _BV(WGM42);
            break;
        case 5:
            TCCR5B = _BV(WGM53) | _BV(WGM52);
            break;
        }        
    }

    void attachInterrupt(void (*isr)()) {
        isrCallback = isr;
	    
        switch (timer_num)
        {
        case 1:
            TIMSK1 = _BV(TOIE1); 
            break;
        case 3:
            TIMSK3 = _BV(TOIE3); 
            break;
        case 4:
            TIMSK4 = _BV(TOIE4); 
            break;
        case 5:
            TIMSK5 = _BV(TOIE5); 
            break;
        }   
    }
    
    void detachInterrupt() {
        switch (timer_num)
        {
        case 1:
            TIMSK1 = 0; 
            break;
        case 3:
            TIMSK3 = 0; 
            break;
        case 4:
            TIMSK4 = 0; 
            break;
        case 5:
            TIMSK5 = 0; 
            break;
        }  
    }

};

extern Timer TimerA;
extern Timer TimerB;
extern Timer TimerC;
extern Timer TimerD;



#endif