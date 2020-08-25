#ifndef ATSAMC21Timer_h
#define ATSAMC21Timer_h

#include "../VirtualTimer.h"
#include <Arduino.h>

class Timer : public VirtualTimer
{
private:
    int pwmPeriod;
    unsigned long timer_resolution;
    unsigned long lastMicroseconds;
public:
    void (*isrCallback)();
    Tcc* timer;

    Timer(Tcc* timer) {
        this->timer = timer;
        if(timer == TCC0 || timer == TCC1) {
            timer_resolution = 16777216;
        } else {
            timer_resolution = 65536;
        }
        lastMicroseconds = 0;
    }

    void initialize() {
        if(timer == TCC0 || timer == TCC1) {
            MCLK->APBCMASK.bit.TCC0_ = 1;
            MCLK->APBCMASK.bit.TCC1_ = 1;
            GCLK->GENCTRL[4].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_OE );
            while ((GCLK->SYNCBUSY.bit.GENCTRL >> 4)  & 1); // Wait for synchronization
            GCLK->PCHCTRL[28].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(4) );  // 28 = TCC0_TCC1
            while ((GCLK->SYNCBUSY.bit.GENCTRL >> 4) & 1); // Wait for synchronization
        } 
        else if (timer == TCC2) {
            MCLK->APBCMASK.bit.TCC2_ = 1;
            GCLK->GENCTRL[5].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_OE );
            while ((GCLK->SYNCBUSY.bit.GENCTRL >> 5)  & 1); // Wait for synchronization
            GCLK->PCHCTRL[29].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(5) ); // 29 = TCC2
            while ((GCLK->SYNCBUSY.bit.GENCTRL >> 5) & 1); // Wait for synchronization
        }
        
        timer->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;             // Select NPWM as waveform
        while (timer->SYNCBUSY.bit.WAVE);                    // Wait for synchronization

    }
    void setPeriod(unsigned long microseconds) {
        if(microseconds == lastMicroseconds)
            return;
        lastMicroseconds = microseconds;
        
        const unsigned long cycles = F_CPU / 1000000 * microseconds;    // cycles corresponds to how many clock ticks per microsecond times number of microseconds we want
        timer->CTRLA.bit.PRESCALER = 0;
        if(cycles < timer_resolution) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);
            pwmPeriod = cycles;
        } else
        if(cycles < timer_resolution * 2) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV2_Val);
            pwmPeriod = cycles / 2;
        } else
        if(cycles < timer_resolution * 4) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV4_Val);
            pwmPeriod = cycles / 4;
        } else
        if(cycles < timer_resolution * 8) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV8_Val);
            pwmPeriod = cycles / 8;
        } else
        if(cycles < timer_resolution * 16) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV16_Val);
            pwmPeriod = cycles / 16;
        } else
        if(cycles < timer_resolution * 64) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV64_Val);
            pwmPeriod = cycles / 64;
        } else
        if(cycles < timer_resolution * 1024) {
            timer->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1024_Val);
            pwmPeriod = cycles / 1024;
        }
        timer->PER.reg = pwmPeriod;
        while (timer->SYNCBUSY.bit.PER);
    }
    void start() {
        timer->CTRLA.bit.ENABLE = 1;                         // Turn on the output
        while (timer->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization
    }
    void stop() {
        timer->CTRLA.bit.ENABLE = 0;                         // Turn on the output
        while (timer->SYNCBUSY.bit.ENABLE);                  // Wait for synchronization
    }

    void attachInterrupt(void (*isr)()) {
        isrCallback = isr;                                  // Store the interrupt callback function
        timer->INTENSET.reg = TCC_INTENSET_OVF;              // Set the interrupt to occur on overflow

        if(timer == TCC0) {
             NVIC_EnableIRQ((IRQn_Type) TCC0_IRQn);              // Enable the interrupt (clock is still off)
        }  
        else if(timer == TCC1) {
             NVIC_EnableIRQ((IRQn_Type) TCC1_IRQn);              // Enable the interrupt (clock is still off)
        }  
        else if(timer == TCC2) {
             NVIC_EnableIRQ((IRQn_Type) TCC2_IRQn);              // Enable the interrupt (clock is still off)
        }  
    }
    
    void detachInterrupt() {
        if(timer == TCC0) {
             NVIC_DisableIRQ((IRQn_Type) TCC0_IRQn);              // Disable the interrupt 
        }  
        else if(timer == TCC1) {
             NVIC_DisableIRQ((IRQn_Type) TCC1_IRQn);              // Disable the interrupt 
        }  
        else if(timer == TCC2) {
             NVIC_DisableIRQ((IRQn_Type) TCC2_IRQn);              // Disable the interrupt 
        }   
    }
};

extern Timer TimerA;
extern Timer TimerB;
extern Timer TimerC;



#endif // ATSAMC21Timer_h
