/*
 * © 2025, Nicola Malavasi. All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC-EX API
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This file implements the Direct Hardware Serial transport layer for 
 * the DFPlayer driver.
 * * KEY FEATURES:
 * 1. Hardware Serial Abstraction: Uses the Arduino HardwareSerial interface 
 * to send and receive data directly via TX/RX pins.
 * 2. Low-Latency Transmission: Overrides transmitCommandBuffer to write 
 * raw packets directly to the serial UART buffer.
 * 3. Asynchronous Receiving: Implements processIncoming using a non-blocking 
 * while loop to drain the serial RX buffer and feed the base byte parser.
 * 4. Integration: Relies on the standard baud rate initialization (9600), 
 * focusing strictly on high-speed data movement.
 * * TECHNICAL NOTE: 
 * On STM32/Nucleo platforms, ensure the use of a 1k Ohm series resistor on 
 * the DFPlayer RX line to mitigate signal noise and electrical mismatch.
 */


#ifndef IO_DFPlayerSerial_h
#define IO_DFPlayerSerial_h

#include "IO_DFPlayerBase.h"

class DFPlayerSerial : public DFPlayerBase {
protected:
    HardwareSerial *_serial;

public:
    DFPlayerSerial(VPIN firstVpin, int nPins, HardwareSerial &s) : DFPlayerBase(firstVpin, nPins) {
        _serial = &s;
        addDevice(this);
    }

    void _begin() override {
        // La seriale viene inizializzata nel bridge IO_DFPlayer.h
        _deviceState = DEVSTATE_NORMAL;
        _display();
        DFPlayerBase::_begin();
    }

    // Implementazione obbligatoria del display per il log
    void _display() override {
        DIAG(F("DFPlayer Serial: Vpin %u %S"), 
             (unsigned int)_firstVpin, 
             (_deviceState==DEVSTATE_FAILED) ? F("FAILED") : F("OK"));
    }

    void transmitCommandBuffer(const uint8_t buffer[], size_t bytes) override {
        if (_deviceState == DEVSTATE_FAILED) return;
        _serial->write(buffer, bytes);
    }

    bool processIncoming() override {
        if (_deviceState == DEVSTATE_FAILED) return false;
        
        bool received = false;
        while (_serial->available()) {
            processIncomingByte(_serial->read());
            received = true;
        }
        return received;
    }
};

#endif