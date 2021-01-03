#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"

void EveryTimerB::isrDefaultUnused(void) {}

// code timer B2. For B0 and B1 copy this code and change the '2' to '0' and '1'
EveryTimerB TimerB2;
ISR(TCB2_INT_vect)
{
  TimerB2.next_tick();
  TCB2.INTFLAGS = TCB_CAPT_bm;
}

#endif // ARDUINO_ARCH_MEGAAVR
