#pragma once

#include "../VirtualTimer.h"
#include <Arduino.h>

class Timer : public VirtualTimer
{
 public:
    Timer(int timer_num) {}

    void initialize()
    {
        //Serial.println("init");
        if (timer != nullptr) stop();
        timer = new IntervalTimer();
    }

    void setPeriod(unsigned long mu)
    {
        //Serial.printf("setPeriod(%d)\n", mu);
        microseconds = mu;
    }
    void start()
    {
        //Serial.printf("start89\n");
        if (timer != nullptr && isrCallback != nullptr)
        {
            timer->begin(isrCallback, microseconds);
        }
        else
        {
            Serial.printf("start error");
        }
    }
    void stop()
    {
        //Serial.printf("stop()\n");
        if (timer != nullptr)
        {
            timer->end();    //this will release the timer, I assume that the lib doesn't call start after stop without initializing.
            delete timer;
            timer = nullptr; // If so -> start would error. In case the lib wants to stop/start something more elaborate needs to be done here
        }
    }

    void attachInterrupt(void (*isr)())
    {
        //Serial.printf("attachInterrupt()\n");
        isrCallback = isr;
    }

    void detachInterrupt()
    {
        stop();
        isrCallback = nullptr;
    }

 private:
    uint32_t microseconds;
    IntervalTimer* timer = nullptr;
    void (*isrCallback)();
};

extern Timer TimerA;
extern Timer TimerB;
extern Timer TimerC;
extern Timer TimerD;
