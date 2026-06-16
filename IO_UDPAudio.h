/*
 * © 2026, Ian Helps. All rights reserved.
 * © 2026 Chris Harlow. All rights reserved.
 * 
 * This file is part of DCC-EX API
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
 
 * ============================================================================
 * IO_UDPAudio.h - Network Audio Broadcast Driver for DCC-EX EXRAIL
 * Version 1.03 - Fixed FOLDER command (folder number in param1, not value)
 * Uses feedback format: <z VPIN> (busy) and <z -VPIN> (idle)
 * ============================================================================
 */

#ifndef IO_UDPAUDIO_H
#define IO_UDPAUDIO_H

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WiFiUDP.h>
#include "IODevice.h"
#include "DIAG.h"
#include "FSH.h"

// DFPlayer Base is used to obtain the opcode constants for the various PLAY commands, to avoid hardcoding them here.
#include "IO_DFPlayerBase.h"


// Per-VPIN state tracking structure
struct AudioNodeState {
    uint8_t currentFolder = 1;
    int currentTrack = 0;
    bool isPlaying = false;
};

class UDPAudio : public IODevice {
public:
    static void create(VPIN firstVpin, int nPins) {
        if (checkNoOverlap(firstVpin, nPins)) {
            new UDPAudio(firstVpin, nPins);
        }
    }

protected:
    void _begin() override {
        _readyToSend = false;
        DIAG(F("NetworkAudio: Ready (lazy init)"));
    }

    // ========================================================================
    // READ: Returns current state for WAITFOR and IF
    // ========================================================================
    int _read(VPIN vpin) override {
        uint8_t idx = vpin - _firstVpin;
        if (idx >= _nPins) return 0;
        return _states[idx].isPlaying ? 1 : 0;
    }

    // ========================================================================
    // WRITE: Handles feedback from node via <z VPIN> and <z -VPIN>
    // Called by: <z VPIN> (positive = BUSY) and <z -VPIN> (negative = IDLE)
    // Also called by EXRAIL SET(VPIN) and RESET(VPIN)
    // NEVER SENDS UDP - only updates internal state
    // ========================================================================
    void _write(VPIN vpin, int state) override {
        uint8_t idx = vpin - _firstVpin;
        if (idx >= _nPins) return;
        
        if (state) return;  // Ignore SET commands
        if (!_states[idx].isPlaying) return;  // Ignore if already idle

        _states[idx].isPlaying = false;
        transmit(vpin, _states[idx].currentTrack, 0, DFPlayerBase::DF_STOPPLAY, _states[idx].currentFolder); // Send STOP command to node
        
        DIAG(F("NetworkAudio: _write() - VPIN %d, isPlaying=%d"), vpin, state);
        // NO UDP BROADCAST HERE!
    }

    // ========================================================================
    // WRITE ANALOGUE: Sends commands to node
    // Called by: PLAY_TRACK, PLAY_PAUSE, PLAY_STOP, PLAY_VOLUME, PLAY_FOLDER
    // THIS IS THE ONLY PLACE THAT SENDS UDP
    // ========================================================================
    void _writeAnalogue(VPIN vpin, int value, uint8_t param1, uint16_t param2) override {
        DIAG(F("NetworkAudio: _writeAnalogue() - vpin=%d, value=%d, param1=%d, param2=%d"), 
             vpin, value, param1, param2);
        
        // Lazy initialization - only when first command sent
        if (WiFi.status() != WL_CONNECTED) return;
        
        if (!_readyToSend) {
            if (_udp.begin(5000)) {
                _readyToSend = true;
                DIAG(F("NetworkAudio: UDP socket initialized on port 5000"));
            } else {
                DIAG(F("NetworkAudio: Failed to initialize UDP socket"));
                return;
            }
        }
        
        uint8_t idx = vpin - _firstVpin;
        if (idx >= _nPins) return;
        
        // Default UDP packet values
        int udpTrack = value;
        int udpVolume = param1;
        int udpFolder = _states[idx].currentFolder;
        
        // ====================================================================
        // Handle different opcodes
        // ====================================================================
        
        switch (param2) {
        
        case DFPlayerBase::DF_FOLDER:
            // IMPORTANT: For PLAY_FOLDER, the folder number is in param1, not value
            _states[idx].currentFolder = param1;
            udpTrack = 0;
            udpVolume = 0;
            udpFolder = param1;
            DIAG(F("NetworkAudio: VPIN %d folder -> %u"), vpin, _states[idx].currentFolder);
            break;
        
        case DFPlayerBase::DF_PLAY: 
            _states[idx].isPlaying = true;
            _states[idx].currentTrack = value;
            DIAG(F("NetworkAudio: VPIN %d PLAY - isPlaying=true"), vpin);
            break;
        
        case DFPlayerBase::DF_RESUME:
            _states[idx].isPlaying = true;
            DIAG(F("NetworkAudio: VPIN %d RESUME - isPlaying=true"), vpin);
            break;
        
        case DFPlayerBase::DF_PAUSE:
            _states[idx].isPlaying = false;
            DIAG(F("NetworkAudio: VPIN %d PAUSE - isPlaying=false"), vpin);
            break;
        
        case DFPlayerBase::DF_STOPPLAY:
            _states[idx].isPlaying = false;
            _states[idx].currentTrack = 0;
            DIAG(F("NetworkAudio: VPIN %d STOP - isPlaying=false"), vpin);
            break;
        
        case DFPlayerBase::DF_VOL:
            // Volume only - state doesn't change
            DIAG(F("NetworkAudio: VPIN %d VOLUME - %u"), vpin, param1);
            break;
            
        case DFPlayerBase::DF_EQ:
            // EQ only - state doesn't change
            DIAG(F("NetworkAudio: VPIN %d EQ - %u"), vpin, param1);
            break;
        case DFPlayerBase::DF_REPEATPLAY:
            return;
            case DFPlayerBase::DF_RESET:
            | `PLAY_RESET(STATION_SPEAKER)` | `PLAY_FOLDER(STATION_SPEAKER, 1)` \+ `PLAY_VOLUME(..., 20)` |

        default:
            // Unknown opcode
            DIAG(F("NetworkAudio: Unknown opcode %d"), param2);
            return;
        }

      transmit(vpin, udpTrack, udpVolume, param2, udpFolder);   
    }

    void _display() override {
        DIAG(F("NetworkAudio: VPINs %u-%u"), 
             (unsigned int)_firstVpin, (unsigned int)_firstVpin + _nPins - 1);
    }

private:
    WiFiUDP _udp;
    AudioNodeState* _states = nullptr;
    bool _readyToSend = false;

    UDPAudio(VPIN firstVpin, int nPins=1) : IODevice(firstVpin, nPins) {
        _states = new AudioNodeState[nPins];
        addDevice(this);
        _display();
    }

    ~UDPAudio() {
        if (_states) {
            delete[] _states;
            _states = nullptr;
        }
    }
    
    void transmit(VPIN vpin, int track, unsigned int volume,
          unsigned int function, unsigned int folder) {
        // ====================================================================
        // Build and send UDP broadcast packet
        // ====================================================================
        char packetBuffer[64];
        IPAddress broadcastIp = WiFi.broadcastIP();
        
        snprintf(packetBuffer, sizeof(packetBuffer), "<a %u %d %u %u %u>", 
                 vpin, track,volume, function, folder);
        _udp.beginPacket(broadcastIp, 5000);
        _udp.write((uint8_t*)packetBuffer, strlen(packetBuffer));
        _udp.endPacket();

        DIAG(F("NetworkAudio Broadcast: %s"), packetBuffer);
    }
};

#endif // ARDUINO_ARCH_ESP32
#endif // IO_UDPAUDIO_H