/****************************************************************************************************************************
  timer.h

  For Portenta_H7 boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Portenta_H7_TimerInterrupt
  Licensed under MIT license

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one Portenta_H7 STM32 timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Version: 1.4.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.2.1   K.Hoang      15/09/2021 Initial coding for Portenta_H7
  1.3.0   K.Hoang      17/09/2021 Add PWM features and examples
  1.3.1   K.Hoang      21/09/2021 Fix warnings in PWM examples
  1.4.0   K.Hoang      22/01/2022 Fix `multiple-definitions` linker error. Fix bug
 *****************************************************************************************************************************/

// Modified from stm32 core v2.0.0

/*
 *******************************************************************************
   Copyright (c) 2019, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIGATIMER_H
#define __GIGATIMER_H
#if defined(ARDUINO_GIGA)
/* Includes ------------------------------------------------------------------*/
#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY)

/* Exported constants --------------------------------------------------------*/
#ifndef TIM_IRQ_PRIO
#if (__CORTEX_M == 0x00U)
#define TIM_IRQ_PRIO       3
#else
#define TIM_IRQ_PRIO       14
#endif /* __CORTEX_M */

#endif /* TIM_IRQ_PRIO */

#ifndef TIM_IRQ_SUBPRIO
#define TIM_IRQ_SUBPRIO    0
#endif

#if defined(TIM1_BASE) && !defined(TIM1_IRQn)
#define TIM1_IRQn TIM1_UP_IRQn
#define TIM1_IRQHandler TIM1_UP_IRQHandler
#endif

#if defined(TIM8_BASE) && !defined(TIM8_IRQn)
#define TIM8_IRQn TIM8_UP_TIM13_IRQn
#define TIM8_IRQHandler TIM8_UP_TIM13_IRQHandler
#endif

#if defined(TIM12_BASE) && !defined(TIM12_IRQn)
#define TIM12_IRQn TIM8_BRK_TIM12_IRQn
#define TIM12_IRQHandler TIM8_BRK_TIM12_IRQHandler
#endif

#if defined(TIM13_BASE) && !defined(TIM13_IRQn)
#define TIM13_IRQn TIM8_UP_TIM13_IRQn
#endif

#if defined(TIM14_BASE) && !defined(TIM14_IRQn)
#define TIM14_IRQn TIM8_TRG_COM_TIM14_IRQn
#define TIM14_IRQHandler TIM8_TRG_COM_TIM14_IRQHandler
#endif


typedef enum
{
#if defined(TIM1_BASE)
  TIMER1_INDEX,
#endif
#if defined(TIM2_BASE)
  TIMER2_INDEX,
#endif
#if defined(TIM3_BASE)
  TIMER3_INDEX,
#endif
#if defined(TIM4_BASE)
  TIMER4_INDEX,
#endif
#if defined(TIM5_BASE)
  TIMER5_INDEX,
#endif
#if defined(TIM6_BASE)
  TIMER6_INDEX,
#endif
#if defined(TIM7_BASE)
  TIMER7_INDEX,
#endif
#if defined(TIM8_BASE)
  TIMER8_INDEX,
#endif
#if defined(TIM9_BASE)
  TIMER9_INDEX,
#endif
#if defined(TIM10_BASE)
  TIMER10_INDEX,
#endif
#if defined(TIM11_BASE)
  TIMER11_INDEX,
#endif
#if defined(TIM12_BASE)
  TIMER12_INDEX,
#endif
#if defined(TIM13_BASE)
  TIMER13_INDEX,
#endif
#if defined(TIM14_BASE)
  TIMER14_INDEX,
#endif
#if defined(TIM15_BASE)
  TIMER15_INDEX,
#endif
#if defined(TIM16_BASE)
  TIMER16_INDEX,
#endif
#if defined(TIM17_BASE)
  TIMER17_INDEX,
#endif
#if defined(TIM18_BASE)
  TIMER18_INDEX,
#endif
#if defined(TIM19_BASE)
  TIMER19_INDEX,
#endif
#if defined(TIM20_BASE)
  TIMER20_INDEX,
#endif
#if defined(TIM21_BASE)
  TIMER21_INDEX,
#endif
#if defined(TIM22_BASE)
  TIMER22_INDEX,
#endif

  TIMER_NUM,
  UNKNOWN_TIMER = 0XFFFF
} timer_index_t;


// This structure is used to be able to get HardwareTimer instance (C++ class)
// from handler (C structure) specially for interrupt management
typedef struct
{
  // Those 2 first fields must remain in this order at the beginning of the structure
  void    *__this;
  TIM_HandleTypeDef handle;
  uint32_t preemptPriority;
  uint32_t subPriority;
} timerObj_t;

/* Exported functions ------------------------------------------------------- */
timerObj_t *get_timer_obj(TIM_HandleTypeDef *htim);

void enableTimerClock(TIM_HandleTypeDef *htim);
void disableTimerClock(TIM_HandleTypeDef *htim);

uint32_t getTimerIrq(TIM_TypeDef *tim);
uint8_t getTimerClkSrc(TIM_TypeDef *tim);

IRQn_Type getTimerUpIrq(TIM_TypeDef *tim);
IRQn_Type getTimerCCIrq(TIM_TypeDef *tim);

#endif /* HAL_TIM_MODULE_ENABLED && !HAL_TIM_MODULE_ONLY */

#ifdef __cplusplus
}
#endif
#endif
#endif /* __GIGATIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
