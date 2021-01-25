#pragma once

#include "../VirtualTimer.h"
#include <Arduino.h>

class Timer : public VirtualTimer
{
 private:
    int pwmPeriod;
    unsigned long timer_resolution;
    unsigned char clockSelectBits;
    int timer_num;
    unsigned long lastMicroseconds;

    uint32_t microseconds;

    IntervalTimer* timer = nullptr;

 public:
    Timer(int timer_num)
    {
        this->timer_num = timer_num; 
        lastMicroseconds = 0;
    }

    void (*isrCallback)();

    void initialize()
    {
        timer = new IntervalTimer();
    }

    void setPeriod(unsigned long mu)
    {
        microseconds = mu;
    }
    void start()
    {
        if(timer != nullptr && isrCallback != nullptr)
        {
            timer->begin(isrCallback, microseconds);
        }
        else
        {
            // some error message to check if the lib calls start without prior intializing
        }
    }
    void stop()
    {
        if(timer != nullptr)
        {
            timer->end();    //this will release the timer, I assume that the lib doesn't start after stop without initializing.
            timer = nullptr; // If so -> start would error. In case the lib wants to stop/start something more elaborate needs to be done here
        }
    }

    void attachInterrupt(void (*isr)())
    {
         isrCallback = isr;
    }

    void detachInterrupt()
    {
        stop();
        isrCallback = nullptr;
    }
};

extern Timer TimerA;
extern Timer TimerB;
extern Timer TimerC;
extern Timer TimerD;
