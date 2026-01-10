
/*
 * © 2026, Nicola Malavasi (NicMal). All rights reserved.
 * © 2025, Nicola Malavasi (NicMal). All rights reserved.
 * © 2023, Neil McKechnie. All rights reserved.
 * * This file is part of DCC++EX API
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
 * This file defines the Abstract Base Class (DFPlayerBase) for DFPlayer devices.
 * It manages the high-level logic, state machine, and command queuing, 
 * independent of the physical transport layer.
 * * KEY FEATURES:
 * 1. Command Queue (FIFO): Manages a 4-slot queue to buffer commands, ensuring 
 * a 120ms delay between transmissions to prevent module lock-ups.
 * 2. Protocol Handling: Automatically calculates the 16-bit checksum and 
 * constructs the 10-byte serial frames required by the DFPlayer hardware.
 * 3. State Management: Tracks playback status (_playing), current volume, 
 * active folder, and loop flags.
 * 4. Bidirectional Feedback: Parses incoming bytes from the module to detect 
 * events like "Track Finished" (0x3D), enabling WAITFOR() synchronization.
 * 5. EX-RAIL Integration: Maps DCC++EX internal opcodes (DF_PLAY, DF_VOL, etc.) 
 * to physical hardware commands via the _writeAnalogue override.
 */


/* * SD CARD FILENAME CONVENTION (Mandatory):
 * ---------------------------------------
 * To ensure compatibility with PLAYSOUND(vpin, track) and DF_FOLDER commands, 
 * the MicroSD must be formatted in FAT32 and follow this naming convention:
 * * 1. FOLDER NAMING:
 * Folders must be named with a 2-digit prefix (01 to 99).
 * Example: /01, /02, /03...
 * * 2. FILE NAMING:
 * Files inside folders must start with a 3-digit prefix (001 to 255).
 * Example: /01/001.mp3, /01/002_Engine_Start.mp3
 * * 3. SPECIAL "MP3" FOLDER (Recommended for simple setups):
 * You can also use a folder named "mp3" (case insensitive) with 
 * 4-digit filenames: /mp3/0001.mp3, /mp3/0002.mp3.
 * * IMPORTANT: The module often plays files based on their physical copy order.
 * It is recommended to clear the SD card and copy files in alphabetical order.
 */



#ifndef IO_DFPlayerBase_h
#define IO_DFPlayerBase_h
#include "IODevice.h"
#include "DIAG.h"

class DFPlayerBase : public IODevice {
public: 
    // Opcode comandi EX-RAIL (Specifiche Chris)
    static const uint8_t  DF_PLAY       =0x0F;
    static const uint8_t  DF_STOPPLAY   =0x16;
    static const uint8_t  DF_REPEATPLAY =0x11;
    static const uint8_t  DF_FOLDER     =0x17; // Flag per cambio cartella
    static const uint8_t  DF_VOL        =0x06;
    static const uint8_t  DF_EQ         =0x07;
    static const uint8_t  DF_RESET      =0x0C;
    static const uint8_t  DF_DACON      =0x1A;
    static const uint8_t  DF_DEBUGON    =0xFD;
    static const uint8_t  DF_DEBUGOFF   =0xFE;
    
    bool debug = false;

protected:
    volatile bool _playing = false;
    bool _flagLoop = false;        
    uint8_t _lastTrack = 1;
    uint8_t _currentFolder = 1;    // Stato persistente
    uint8_t _currentVolume = 20;   // Stato persistente
    uint8_t _inputIndex = 0;
    uint8_t _recvCMD = 0; 
    unsigned long _lastXmit = 0;
  
    struct CommandEntry { uint8_t cmd; uint8_t a1; uint8_t a2; };
    static const uint8_t Q_SIZE = 4;
    CommandEntry _q[Q_SIZE];
    uint8_t _head = 0; uint8_t _tail = 0;  

    void queuePacket(uint8_t c, uint8_t a1 = 0, uint8_t a2 = 0) {
        uint8_t n = (_head + 1) % Q_SIZE;
        if (n != _tail) { _q[_head] = {c, a1, a2}; _head = n; }
    }

    DFPlayerBase(VPIN firstVpin, int nPins=1): IODevice(firstVpin, nPins) {} 

public:
    void _begin() override { _deviceState = DEVSTATE_INITIALISING; }

    // Subclasses must implement these transport-specific methods
    virtual void transmitCommandBuffer(const uint8_t buffer[], size_t bytes) = 0; 
    virtual bool processIncoming() = 0; 

    void _loop(unsigned long currentMicros) override {
        if (_deviceState == DEVSTATE_FAILED) return;
        processIncoming(); 

        if (millis() - _lastXmit > 120 && _head != _tail) {
            // Data to send, construct packet.
            uint8_t out[] = {0x7E, 0xFF, 0x06, _q[_tail].cmd, 0x00, _q[_tail].a1, _q[_tail].a2, 0x00, 0x00, 0xEF};
            int16_t sum = 0; for (int i = 1; i < 7; i++) sum -= out[i];
            out[7] = (uint8_t)(sum >> 8); out[8] = (uint8_t)(sum & 0xff);
            transmitCommandBuffer(out, 10);
            _tail = (_tail + 1) % Q_SIZE;
            _lastXmit = millis();
        }
        delayUntil(currentMicros + 15000); 
    }

    void processIncomingByte(byte c) {
        if (debug) DIAG(F("DFPlayer RX Byte(%d): 0x%x"), _inputIndex,c);
        static const byte HDR[] = {0x7E, 0xFF, 0x06};
        if (_inputIndex < 3) {
            if (c == HDR[_inputIndex]) _inputIndex++; else _inputIndex = 0;
            return;
        }
        if (_inputIndex == 3) _recvCMD = c;
        if (_inputIndex == 6 && _recvCMD == 0x3D) { 
            if (_flagLoop) { 
                queuePacket(DF_PLAY, _currentFolder, _lastTrack); 
            } else {
                _playing = false; 
            }
        }
        _inputIndex++;
        if (_inputIndex >= 10) _inputIndex = 0; 
    }

    // HAL interface for SET/RESET digital write
    void _write(VPIN vpin, int value) override {
        _flagLoop = false; 
        if (value) { 
            _playing = true; 
            _lastTrack = (uint8_t)(vpin - _firstVpin + 1);
            queuePacket(DF_PLAY, _currentFolder, _lastTrack); 
        } else { 
            _playing = false; 
            queuePacket(DF_STOPPLAY, 0, 0); 
        }
    }

    // HAL Interface for digital read, WAITFOR 
    int _read(VPIN vpin) override { return _playing ? 1 : 0; }

protected:
    void _writeAnalogue(VPIN vpin, int v1, uint8_t v2=0, uint16_t cmd=0) override {
        if (_deviceState == DEVSTATE_FAILED) return;

        switch (cmd){
            case DF_FOLDER:
                _currentFolder = (v2 > 0) ? v2 : 1; 
                break;

            case DF_VOL:
                _currentVolume = (v2 > 0) ? v2 : 20;
                queuePacket(0x06, 0x00, _currentVolume); 
                break;

            case DF_EQ:
                queuePacket(0x07, 0x00, (uint8_t)v2); 
                break;

            case DF_RESET:
                queuePacket(0x0C, 0x00, 0x00);
                _currentFolder = 1;
                _currentVolume = 30; // Default hardware dopo reset
                break;

            case DF_DACON:
                queuePacket(0x1A, 0x00, (v2 > 0) ? 0x01 : 0x00);
                break;

            case DF_PLAY:
            case DF_REPEATPLAY:
                _flagLoop = (cmd == DF_REPEATPLAY);
                _playing = true; 
                _lastTrack = (uint8_t)v1;
                // Se v2 è fornito, aggiorna il volume prima del play
                if (v2 > 0) {
                    _currentVolume = v2;
                    queuePacket(0x06, 0x00, _currentVolume);
                }
                queuePacket(DF_PLAY, _currentFolder, _lastTrack); 
                break;

            case DF_STOPPLAY:
                _flagLoop = false; _playing = false;
                queuePacket(DF_STOPPLAY, 0, 0); 
                break;
            case DF_DEBUGON:
                debug = true;
                break;
            case DF_DEBUGOFF:
                debug = false;
                break;
        }
    }  
    
    void dumpBuffer(const FSH* label, const uint8_t buf[], size_t len) {
        USB_SERIAL.print(F("<* "));
        USB_SERIAL.print(label);
        for (size_t i = 0; i < len; i++) {
            USB_SERIAL.print((buf[i] < 0x10) ? F(" 0"):F(" "));
            USB_SERIAL.print(buf[i],HEX);
        }
        USB_SERIAL.println(F(" *>"));
    }
};
#endif