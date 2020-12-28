#if !defined(MegaAvr20MHz_h_)
#define MegaAvr20MHz_h_
#if defined(ARDUINO_ARCH_MEGAAVR) && (F_CPU == 20000000UL) && defined(MILLIS_USE_TIMERB3)
#define MegaAvr20MHzCorrected
// Quick hack to correct the millis() and micros() functions for 20MHz MegaAVR boards.
// by Kees van der Oord <Kees.van.der.Oord@inter.nl.net>
// Remember to call the function corrected20MHzInit() from setup() or an class constructor !

// in the IDE 1.8.5 the implementation of millis() and micros() is not accurate
// for the MegaAvr achitecture board clocked at 20 MHz:
// 1)
// in ~\Arduino15\packages\arduino\hardware\megaavr\1.8.5\cores\arduino\wiring.c(386)
// microseconds_per_timer_overflow is initialized as:
// microseconds_per_timer_overflow = clockCyclesToMicroseconds(TIME_TRACKING_CYCLES_PER_OVF);
// this evaluates to (256 * 64) / (20000000/1000000)) = 819.2 which is rounded 819.
// the rounding causes millis() and micros() to report times that are 0.2/819.2 = 0.024 % too short 
// 2)
// in ~\Arduino15\packages\arduino\hardware\megaavr\1.8.5\cores\arduino\wiring.c(387)
// microseconds_per_timer_tick is defined as:
// microseconds_per_timer_tick = microseconds_per_timer_overflow/TIME_TRACKING_TIMER_PERIOD;
// which evaluates to 819.2 / 255 = 3.21254901960784 which is rounded to 3
// this is wrong in two ways:
// - the TIME_TRACKING_TIMER_PERIOD constant is wrong: this should be TIME_TRACKING_TICKS_PER_OVF
//   so the correct value is 3.2 ns/tick
// - the rounding causes micros() to return times that are 0.2/3 = 6.25 % too short
// as a quick hack, initialize these variables with settings a factor 5 larger
// and redefine the millis() and  micros() functions to return the corrected values

// The code in this header file corrects for these problems by incrementing the counters
// with increments that are 5 times larger (the lowest factor that gives integer values).
// The millis() and micros() functions are redefined to return the counters / 5.
// The costs you pay is that the number of clock cycles of the new millis() and micros() 
// functions is higher. This should be covered by the fact that the chip runs 25% faster
// at 20 MHz than at 16 MHz.

// This header file redefines the millis() and micros() functions. The redefinition
// is only active for source files in which this header file is included. If you link
// to libraries with a .cpp file, you have to manually change the library .cpp file to 
// include this header as well. In addition the corrected20MHzInit() method must be called
// from your sketch to re-initialize the variables used by the timer isr function.

// for micros()
// from wiring.c:
extern volatile uint32_t timer_overflow_count;

inline unsigned long corrected_micros() {
  
  static volatile unsigned long microseconds_offset = 0;

  unsigned long overflows, microseconds;
  uint8_t ticks;
  unsigned long offset;

  // Save current state and disable interrupts 
  uint8_t status = SREG;
  cli();

  // we need to prevent that the double calculation below exceeds MAX_ULONG 
  // this assumes that micros() is called at least once every 35mins)
  while(timer_overflow_count > 500000UL) {
    microseconds_offset += 409600000UL; // 500000 * 819.2 ~ almost 7 minutes
    timer_overflow_count -= 500000UL;
  }

  // Get current number of overflows and timer count
  overflows = timer_overflow_count;
  ticks = TCB3.CNTL;
  offset = microseconds_offset; 

  // If the timer overflow flag is raised, we just missed it,
  // increment to account for it, & read new ticks 
  if(TCB3.INTFLAGS & TCB_CAPT_bm){
    overflows++;
    ticks = TCB3.CNTL;
  }

  // Restore state
  SREG = status;

  // Return microseconds of up time  (resets every ~70mins) 
  // float aritmic is faster than integer multiplication ? 
  return offset + (unsigned long)((overflows * 819.2) + (ticks * 3.2));
}
#define micros corrected_micros

// for millis()
// from wiring.c:
extern volatile uint32_t timer_millis;
extern uint16_t millis_inc;
extern uint16_t fract_inc;

// call this method from your sketch setup() if you include this file !
inline void corrected20MHzInit(void) {
  fract_inc = 96; // (5 * 819.2) % 1000
  millis_inc = 4; // (5 * 819.2) / 1000
}

inline unsigned long corrected_millis() {
  static volatile unsigned long last = 0;
  static volatile unsigned long integer = 0;
  static volatile unsigned long fraction = 0;

  unsigned long m;

  // disable interrupts while we read timer_millis or we might get an
  // inconsistent value (e.g. in the middle of a write to timer_millis)
  uint8_t status = SREG;
  cli();

  unsigned long elapsed = timer_millis - last;
  last = timer_millis;
  integer += elapsed / 5;
  fraction += elapsed % 5;
  if(fraction >= 5) { ++integer; fraction -= 5; }

  m = integer;
  
  SREG = status;
  
  return m;
}
#define millis corrected_millis

#endif // defined(ARDUINO_ARCH_MEGAAVR) && (F_CPU == 20000000UL) && defined(MILLIS_USE_TIMERB3)

#endif // !defined(MegaAvr20MHz_h_)
