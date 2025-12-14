/*
 *  © 2023, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

/*
 * DFPlayerSerial is a subclass of DFPlayerBase that implements communication
 * to the DFPlayer module using a HardwareSerial port.
 * This driver writes buffers to the srial and passes incoming bytes to the
 * base class for processing.
 * */

#ifndef IO_DFPlayerSerial_h
#define IO_DFPlayerSerial_h

#include "IODevice.h"
#include "IO_DFPlayerBase.h"

class DFPlayerSerial: public DFPlayerBase {
private: 
  HardwareSerial *_serial;
  
public:

  // Constructor
  DFPlayerSerial(VPIN firstVpin, int nPins, HardwareSerial &serial) : 
    DFPlayerBase(firstVpin, nPins),
    _serial(&serial) 
  {
    addDevice(this);
  }


  void transmitCommandBuffer(const uint8_t buffer[],size_t bytes) override {
      // Transmit the command buffer via the serial port
        _serial->write(buffer,bytes);
    }

  bool processIncoming() override  {
      bool found=false;
      while(_serial->available()) {
        processIncomingByte(_serial->read());
        found=true;
        }
        return found;
      }
    };
#endif // IO_DFPlayerSerial_h
