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

#include "defines.h" // includes config.h
#ifndef DISABLE_EEPROM
#include "EEStore.h"
#endif

#include "PinServoTurnout.h"

/*************************************************************************************
 * PinServoTurnout - Turnout controlled by servo device connected to a processor pin.
 *
 *************************************************************************************/

// Private Constructor
PinServoTurnout::PinServoTurnout(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed) : Turnout(id, TURNOUT_SERVO, closed)
{
  _servoTurnoutData.vpin = vpin;
  _servoTurnoutData.thrownPosition = thrownPosition;
  _servoTurnoutData.closedPosition = closedPosition;
  _servoTurnoutData.profile = profile;
}

// Create function
/* static */ Turnout *PinServoTurnout::create(uint16_t id, VPIN vpin, uint16_t thrownPosition, uint16_t closedPosition, uint8_t profile, bool closed)
{
#ifndef IO_NO_HAL
  Turnout *tt = get(id);
  if (tt)
  {
    // Object already exists, check if it is usable
    if (tt->isType(TURNOUT_PIN))
    {
      // Yes, so set parameters
      PinServoTurnout *st = (PinServoTurnout *)tt;
      st->_servoTurnoutData.vpin = vpin;
      st->_servoTurnoutData.thrownPosition = thrownPosition;
      st->_servoTurnoutData.closedPosition = closedPosition;
      st->_servoTurnoutData.profile = profile;
      // Don't touch the _closed parameter, retain the original value.

      st->servo.attach(vpin);
      st->servo.write(closed ? closedPosition : thrownPosition);

      return tt;
    }
    else
    {
      // Incompatible object, delete and recreate
      remove(id);
    }
  }
  PinServoTurnout* ttnew = new PinServoTurnout(id, vpin, thrownPosition, closedPosition, profile, closed);
  DIAG(F("PinServoTurnout 0x%x size %d size %d"), tt, sizeof(Turnout), sizeof(struct TurnoutData));

  ttnew->servo.attach(vpin);
  ttnew->servo.write(closed ? closedPosition : thrownPosition);
  return (Turnout *)ttnew;
#else
  (void)id;
  (void)vpin;
  (void)thrownPosition;
  (void)closedPosition;
  (void)profile;
  (void)closed; // avoid compiler warnings.
  return NULL;
#endif
}

// Load a Pin Servo turnout definition from EEPROM.  The common Turnout data has already been read at this point.
Turnout *PinServoTurnout::load(struct TurnoutData *turnoutData)
{
#ifndef DISABLE_EEPROM
  PinServoTurnoutData servoTurnoutData;
  // Read class-specific data from EEPROM
  EEPROM.get(EEStore::pointer(), servoTurnoutData);
  EEStore::advance(sizeof(servoTurnoutData));

  // Create new object
  Turnout *tt = PinServoTurnout::create(turnoutData->id, servoTurnoutData.vpin, servoTurnoutData.thrownPosition,
                                        servoTurnoutData.closedPosition, servoTurnoutData.profile, turnoutData->closed);
  return tt;
#else
  (void)turnoutData;
  return NULL;
#endif
}

// For DCC++ classic compatibility, state reported to JMRI is 1 for thrown and 0 for closed
void PinServoTurnout::print(Print *stream)
{
  StringFormatter::send(stream, F("<H %d SERVO %d %d %d %d %d>\n"), _turnoutData.id, _servoTurnoutData.vpin,
                        _servoTurnoutData.thrownPosition, _servoTurnoutData.closedPosition, _servoTurnoutData.profile,
                        !_turnoutData.closed);
}

// ServoTurnout-specific code for throwing or closing a servo turnout.
bool PinServoTurnout::setClosedInternal(bool close)
{
  servo.write(close ? _servoTurnoutData.closedPosition : _servoTurnoutData.thrownPosition);
  return true;
}

void PinServoTurnout::save()
{
#ifndef DISABLE_EEPROM
  // Write turnout definition and current position to EEPROM
  // First write common servo data, then
  // write the servo-specific data
  EEPROM.put(EEStore::pointer(), _turnoutData);
  EEStore::advance(sizeof(_turnoutData));
  EEPROM.put(EEStore::pointer(), _servoTurnoutData);
  EEStore::advance(sizeof(_servoTurnoutData));
#endif
}
