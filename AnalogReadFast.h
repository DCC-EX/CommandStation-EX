/*
 *  AnalogReadFast.h
 * 
 *  Copyright (C) 2016  Albert van Dalen http://www.avdweb.nl
 * 
 *  This file is part of CommandStation.
 *
 *  CommandStation is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMANDSTATION_DCC_ANALOGREADFAST_H_
#define COMMANDSTATION_DCC_ANALOGREADFAST_H_

#include <Arduino.h>

int inline analogReadFast(uint8_t ADCpin);

#if defined(ARDUINO_ARCH_SAMD) 
int inline analogReadFast(uint8_t ADCpin)
{ ADC->CTRLA.bit.ENABLE = 0;              // disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 ); // wait for synchronization

  int CTRLBoriginal = ADC->CTRLB.reg;
  int AVGCTRLoriginal = ADC->AVGCTRL.reg;
  int SAMPCTRLoriginal = ADC->SAMPCTRL.reg;
  
  ADC->CTRLB.reg &= 0b1111100011111111;          // mask PRESCALER bits
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV64;   // divide Clock by 64
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // take 1 sample 
                     ADC_AVGCTRL_ADJRES(0x00ul); // adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // sampling Time Length = 0

  ADC->CTRLA.bit.ENABLE = 1;                     // enable ADC
  while(ADC->STATUS.bit.SYNCBUSY == 1);          // wait for synchronization

  int adc = analogRead(ADCpin); 
  
  ADC->CTRLB.reg = CTRLBoriginal;
  ADC->AVGCTRL.reg = AVGCTRLoriginal;
  ADC->SAMPCTRL.reg = SAMPCTRLoriginal;
   
  return adc;
}

#elif defined(ARDUINO_ARCH_SAMC) 

int inline analogReadFast(uint8_t ADCpin)
{ 
  Adc* ADC;
  if ( (g_APinDescription[ADCpin].ulPeripheralAttribute & PER_ATTR_ADC_MASK) == PER_ATTR_ADC_STD ) {
    ADC = ADC0;
  } else {
    ADC = ADC1;
  }
  
  ADC->CTRLA.bit.ENABLE = 0;              // disable ADC
  while( ADC->SYNCBUSY.bit.ENABLE == 1 ); // wait for synchronization

  int CTRLBoriginal = ADC->CTRLB.reg;
  int AVGCTRLoriginal = ADC->AVGCTRL.reg;
  int SAMPCTRLoriginal = ADC->SAMPCTRL.reg;
  
  ADC->CTRLB.reg &= 0b1111100011111111;          // mask PRESCALER bits
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV64;   // divide Clock by 64
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // take 1 sample 
                     ADC_AVGCTRL_ADJRES(0x00ul); // adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // sampling Time Length = 0

  ADC->CTRLA.bit.ENABLE = 1;                     // enable ADC
  while(ADC->SYNCBUSY.bit.ENABLE == 1);          // wait for synchronization

  int adc = analogRead(ADCpin); 
  
  ADC->CTRLB.reg = CTRLBoriginal;
  ADC->AVGCTRL.reg = AVGCTRLoriginal;
  ADC->SAMPCTRL.reg = SAMPCTRLoriginal;
   
  return adc;
}

#elif defined(ARDUINO_AVR_UNO_WIFI_REV2) || defined(ARDUINO_AVR_NANO_EVERY)

int inline analogReadFast(uint8_t ADCpin) 
{ byte ADC0CTRLCoriginal = ADC0.CTRLC; 
  ADC0.CTRLC = (ADC0CTRLCoriginal & 0b00110000) + 0b01000011; 
  int adc = analogRead(ADCpin);  
  ADC0.CTRLC = ADC0CTRLCoriginal;
  return adc;
}

#else

int inline analogReadFast(uint8_t ADCpin) 
{ byte ADCSRAoriginal = ADCSRA; 
  ADCSRA = (ADCSRA & B11111000) | 4; 
  int adc = analogRead(ADCpin);  
  ADCSRA = ADCSRAoriginal;
  return adc;
}
#endif
#endif  // COMMANDSTATION_DCC_ANALOGREADFAST_H_