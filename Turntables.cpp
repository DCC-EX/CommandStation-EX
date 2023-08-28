/*
 *  © 2023 Peter Cole
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

#include "defines.h"
#include <Arduino.h>
#include "Turntables.h"
#include "StringFormatter.h"
#include "CommandDistributor.h"
#include "EXRAIL2.h"
#include "DCC.h"


/*
 * Protected static data
 */
Turntable *Turntable::_firstTurntable = 0;


/*
 * Public static data
 */
int Turntable::turntablelistHash = 0;


/*
 * Protected static functions
 */
// Add new turntable to end of list

void Turntable::add(Turntable *tto) {
  if (!_firstTurntable) {
    _firstTurntable = tto;
  } else {
    Turntable *ptr = _firstTurntable;
    for ( ; ptr->_nextTurntable!=0; ptr=ptr->_nextTurntable) {}
    ptr->_nextTurntable = tto;
  }
  turntablelistHash++;
}

// Find turntable from list
Turntable *Turntable::get(uint16_t id) {
  for (Turntable *tto = _firstTurntable; tto != NULL; tto = tto->_nextTurntable)
    if (tto->_turntableData.id == id) return tto;
  return NULL;
}

// Add a position
void Turntable::addPosition(uint16_t value) {
  _turntablePositions.insert(value);
}

// Get value for position
uint16_t Turntable::getPositionValue(size_t position) {
  TurntablePosition* currentPosition = _turntablePositions.getHead();
  for (size_t i = 0; i < position && currentPosition; i++) {
    currentPosition = currentPosition->next;
  }

  if (currentPosition) {
    return currentPosition->data;
  } else {
    return false;
  }
}

/*
 * Public static functions
 */
bool Turntable::setPositionStateOnly(uint16_t id, uint8_t position) {
  Turntable *tto = get(id);
  if (!tto) return false;
  CommandDistributor::broadcastTurntable(id, position);
#if defined(EXRAIL_ACTIVE)
  // RMFT2::turntableEvent(id, position);
#endif
  return true;
}

bool Turntable::setPosition(uint16_t id, uint8_t position, uint8_t activity) {
#if defined(DIAG_IO)
  DIAG(F("Turntable(%d, %d)"), id, position);
#endif
  Turntable *tto = Turntable::get(id);
  if (!tto) return false;
  bool ok = tto->setPositionInternal(position, activity);

  if (ok) {
    tto->setPositionStateOnly(id, position);
    tto->_turntableData.position = position;
  }
  return ok;
}

/*************************************************************************************
 * EXTTTurntable - EX-Turntable device.
 * 
 *************************************************************************************/
// Private constructor
EXTTTurntable::EXTTTurntable(uint16_t id, VPIN vpin, uint8_t i2caddress) :
  Turntable(id, TURNTABLE_EXTT)
{
  _exttTurntableData.vpin = vpin;
  _exttTurntableData.i2caddress = i2caddress;
}

// Create function
  Turntable *EXTTTurntable::create(uint16_t id, VPIN vpin, uint8_t i2caddress) {
#ifndef IO_NO_HAL
    Turntable *tto = get(id);
    if (tto) {
      if (tto->isType(TURNTABLE_EXTT)) {
        EXTTTurntable *extt = (EXTTTurntable *)tto;
        extt->_exttTurntableData.vpin = vpin;
        extt->_exttTurntableData.i2caddress = i2caddress;
        return tto;
      }
    }
    tto = (Turntable *)new EXTTTurntable(id, vpin, i2caddress);
    DIAG(F("Turntable 0x%x"), tto);
    return tto;
#else
  (void)id;
  (void)i2caddress;
  (void)vpin;
  return NULL;
#endif
  }

  void EXTTTurntable::print(Print *stream) {
    StringFormatter::send(stream, F("<i %d EXTURNTABLE %d %d>\n"), _turntableData.id, _exttTurntableData.vpin, _exttTurntableData.i2caddress);
  }

  // EX-Turntable specific code for moving to the specified position
  bool EXTTTurntable::setPositionInternal(uint8_t position, uint8_t activity) {
#ifndef IO_NO_HAL
    DIAG(F("Set EXTT %d to position %d with activity %d"), _exttTurntableData.vpin, position, activity);
    if (position == 0) return false;  // Position 0 is just so throttles know where home is
    int16_t value = getPositionValue(position); // Get position value from position list
    if (!value) return false; // Return false if it's not a valid position
    // Set position via device driver
    EXTurntable::writeAnalogue(_exttTurntableData.vpin, value, activity);
#else
    (void)position;
#endif
    return true;
  }
