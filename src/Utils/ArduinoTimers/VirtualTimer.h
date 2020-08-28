/*
 *  VirtualTimer.h
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

#ifndef UTILS_ARDUINOTIMERS_VIRTUALTIMER_H_
#define UTILS_ARDUINOTIMERS_VIRTUALTIMER_H_

class VirtualTimer
{
public:
  virtual void initialize() = 0;
  virtual void setPeriod(unsigned long microseconds) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;

  virtual void attachInterrupt(void (*isr)()) = 0;
  virtual void detachInterrupt() = 0;
private:

};

#endif  // UTILS_ARDUINOTIMERS_VIRTUALTIMER_H_