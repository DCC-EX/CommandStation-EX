/*
 *  © 2023 Andrey Baboshin
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef IO_PINSERVO_H
#define IO_PINSERVO_H

#include "Turnouts.h"
#include <Servo.h>

class PinServoTurnout : public Turnout
{
private:
  // PinServoTurnoutData contains data specific to this subclass that is
  // written to EEPROM when the turnout is saved.
  struct PinServoTurnoutData
  {
    VPIN vpin;
    uint16_t closedPosition;
    uint16_t thrownPosition;
    uint8_t profile;
  } _servoTurnoutData; // 6 bytes

  Servo servo;

  // Constructor
  PinServoTurnout(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed);

public:
  // Create function
  static Turnout *create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed = true);

  // Load a Servo turnout definition from EEPROM.  The common Turnout data has already been read at this point.
  static Turnout *load(struct TurnoutData *turnoutData);
  void print(Print *stream) override;

protected:
  // PinServoTurnout-specific code for throwing or closing a servo turnout.
  bool setClosedInternal(bool close) override;
  void save() override;
};

#endif // IO_PINSERVO_H
