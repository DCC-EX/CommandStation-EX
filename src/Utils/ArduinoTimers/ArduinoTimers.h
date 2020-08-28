/*
 *  ArduinoTimers.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UTILS_ARDUINOTIMERS_ARDUINOTIMERS_H_
#define UTILS_ARDUINOTIMERS_ARDUINOTIMERS_H_

#if defined(ARDUINO_ARCH_SAMC)
  #include "ATSAMC21/Timer.h"
#elif defined(ARDUINO_ARCH_SAMD)
  #include "ATSAMD21/Timer.h"
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  #include "ATMEGA2560/Timer.h"
#elif defined(ARDUINO_AVR_UNO)
  #include "ATMEGA328/Timer.h"
#elif defined(ARDUINO_ARCH_MEGAAVR)
  #include "ATMEGA4809/Timer.h"
#else
  #error "Cannot compile - ArduinoTimers library does not support your board, or you are missing compatible build flags."
#endif

#endif  // UTILS_ARDUINOTIMERS_ARDUINOTIMERS_H_
