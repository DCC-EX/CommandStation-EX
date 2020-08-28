/*
 *  CommInterface.h
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

#ifndef COMMINTERFACE_COMMINTERFACE_H_
#define COMMINTERFACE_COMMINTERFACE_H_

#include <Stream.h>

class CommInterface {
public:
  virtual void process() = 0;
  virtual void showConfiguration() = 0;
  virtual void showInitInfo() = 0;

#if defined(ARDUINO_ARCH_MEGAAVR)
  virtual arduino::Print* getStream() = 0;
#else
  virtual Print* getStream() = 0;
#endif

};

#endif	// COMMINTERFACE_COMMINTERFACE_H_