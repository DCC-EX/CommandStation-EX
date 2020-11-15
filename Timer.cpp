// This file is copied from https://github.com/davidcutting42/ArduinoTimers
// All Credit to David Cutting

#include <Arduino.h>

#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)

#include "ATMEGA2560/Timer.h"

Timer TimerA(1);
Timer TimerB(3);
Timer TimerC(4);
Timer TimerD(5);

ISR(TIMER1_OVF_vect)
{
    TimerA.isrCallback();
}

ISR(TIMER3_OVF_vect)
{
    TimerB.isrCallback();
}

ISR(TIMER4_OVF_vect)
{
    TimerC.isrCallback();
}

ISR(TIMER5_OVF_vect)
{
    TimerD.isrCallback();
}

#elif defined(ARDUINO_AVR_UNO)      // Todo: add other 328 boards for compatibility

#include "ATMEGA328/Timer.h"

Timer TimerA(1);
Timer TimerB(2);

ISR(TIMER1_OVF_vect)
{
    TimerA.isrCallback();
}

ISR(TIMER2_OVF_vect)
{
    TimerB.isrCallback();
}

#endif
