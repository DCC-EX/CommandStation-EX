#ifndef ATSAMD21GTimer_h
#define ATSAMD21GTimer_h

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
            REG_GCLK_GENDIV =   GCLK_GENDIV_DIV(1) |            // Divide 48MHz by 1
                                GCLK_GENDIV_ID(4);              // Apply to GCLK4
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
                
            REG_GCLK_GENCTRL =  GCLK_GENCTRL_GENEN |            // Enable GCLK
                                GCLK_GENCTRL_SRC_DFLL48M |      // Set the 48MHz clock source
                                GCLK_GENCTRL_ID(4);             // Select GCLK4
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
            
            REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_CLKEN |            // Enable generic clock
                                4 << GCLK_CLKCTRL_GEN_Pos |     // Apply to GCLK4
                                GCLK_CLKCTRL_ID_TCC0_TCC1;      // Feed GCLK to TCC0/1
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
        } 
        else if (timer == TCC2) {
            REG_GCLK_GENDIV =   GCLK_GENDIV_DIV(1) |            // Divide 48MHz by 1
                                GCLK_GENDIV_ID(5);              // Apply to GCLK4
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
                
            REG_GCLK_GENCTRL =  GCLK_GENCTRL_GENEN |            // Enable GCLK
                                GCLK_GENCTRL_SRC_DFLL48M |      // Set the 48MHz clock source
                                GCLK_GENCTRL_ID(5);             // Select GCLK4
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
            
            REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_CLKEN |            // Enable generic clock
                                5 << GCLK_CLKCTRL_GEN_Pos |     // Apply to GCLK4
                                GCLK_CLKCTRL_ID_TCC2_TC3;      // Feed GCLK to TCC0/1
            while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
        }
        
        
        timer->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;             // Select NPWM as waveform
        while (timer->SYNCBUSY.bit.WAVE);                    // Wait for synchronization

    }
    void setPeriod(unsigned long microseconds) {
        if(microseconds == lastMicroseconds)
            return;
        lastMicroseconds = microseconds;
        
        const unsigned long cycles = F_CPU / 1000000 * microseconds;    // cycles corresponds to how many clock ticks per microsecond times number of microseconds we want
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



#endif