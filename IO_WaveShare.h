
/*
 *  © 2026 Ian Helps
 *  © 2026 Chris Harlow
 *  
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
#ifndef IO_WAVESHARE_H
#define IO_WAVESHARE_H

#if __has_include(<AudioTools.h>)

#include <Arduino.h>
#include <Preferences.h>

#include "defines.h"
#include "IODevice.h"

#define TCA9555 TCA9555_AUDIOTOOLS
#include <AudioBoards/ESP32S3AISmartSpeaker.h>
#include <AudioTools.h>
#include <AudioTools/AudioCodecs/CodecMP3Helix.h>
#include <AudioTools/AudioLibs/AudioBoardStream.h>
#include <AudioTools/Disk/AudioSourceSDFAT.h>
#undef TCA9555

#include "DIAG.h"
#include "IO_PCA9555.h"

// The WaveShare board routes SD chip select through the TCA9555 expander.
// AudioSourceSDFAT is constructed with cs=-1 for this board, so we override
// SdFat's weak CS hooks to avoid touching pin 255 and drive EXIO4 instead.
void sdCsInit(SdCsPin_t pin) {
    (void)pin;
}

void sdCsWrite(SdCsPin_t pin, bool level) {
    (void)pin;
    audio_driver::PinsESP32S3AISmartSpeaker.getTCA9555().digitalWrite(EXIO4, level);
}

class WaveShare : public IODevice {
public:
    static void create(VPIN firstVpin) {
        if (checkNoOverlap(firstVpin, 1)) {
            new WaveShare(firstVpin);
        }
    }

    // CAUTION, these constants are copied from DFPlayerBase, and must be kept in sync with that class.
    static const uint8_t DF_PLAY       = 0x0F;
    static const uint8_t DF_STOPPLAY   = 0x16;
    static const uint8_t DF_REPEATPLAY = 0x11;
    static const uint8_t DF_FOLDER     = 0x17;
    static const uint8_t DF_VOL        = 0x06;
    static const uint8_t DF_EQ         = 0x07;
    static const uint8_t DF_RESET      = 0x0C;
    static const uint8_t DF_PAUSE      = 0x0E;
    static const uint8_t DF_RESUME     = 0x0D;

    static const uint8_t DF_EQ_NORMAL  = 0;
    static const uint8_t DF_EQ_POP     = 1;
    static const uint8_t DF_EQ_ROCK    = 2;
    static const uint8_t DF_EQ_JAZZ    = 3;
    static const uint8_t DF_EQ_CLASSIC = 4;
    static const uint8_t DF_EQ_BASS    = 5;

protected:
    bool _repeat = false;
    uint8_t _currentFolder = 1;
    uint8_t _currentTrack = 0;

    WaveShare(VPIN firstVpin) : IODevice(firstVpin, 1) {
        source = new AudioSourceSDFAT<SdFat32, File32>("", "mp3", -1, 2);
        decoder = new MP3DecoderHelix();
        kit = new AudioBoardStream(ESP32S3AISmartSpeaker);
        player = new AudioPlayer(*source, *kit, *decoder);
        addDevice(this);
    }

public:
    void _begin() override {
        DIAG(F("WaveShare::_begin()"));

        _currentVolume = 0.7f;

        // Setup audio output
        auto cfg = kit->defaultConfig(TX_MODE);
        cfg.sd_active = true;
        bool boardReady = kit->begin(cfg);
        USB_SERIAL.printf("Audio board begin: %s\n", boardReady ? "OK" : "FAIL");
        USB_SERIAL.println("Initializing SD card at 2 MHz...");

        // Setup player callbacks (before SD card)
        player->stop();
        player->setVolume(0.7f);
        player->setAutoNext(false);

        if (!source->begin()) {
            DIAG(F("ERROR: Failed to initialize SD card!\n"
                   "Check that SD card is inserted and formatted FAT32\n"
                   "SDFAT error code: 0x%x data: 0x%x\n"),
                 source->getAudioFs().sdErrorCode(),
                 source->getAudioFs().sdErrorData());
        }

        USB_SERIAL.print("<* SD card initialized\n");
        listSDFiles("/");
        USB_SERIAL.print(" *>\n");

        _display();
    }

    void _display() override {
        USB_SERIAL.printf("<* WaveShare Audio Device VPIN %d:\n", _firstVpin);
        USB_SERIAL.printf("  Current Volume: %.2f\n", _currentVolume);
        USB_SERIAL.printf("  Current File: %d/%d\n", _currentFolder, _currentTrack);
        USB_SERIAL.printf("  Is Playing: %s *>\n", player->isActive() ? "Yes" : "No");
    }

    void _loop(unsigned long currentMicros) override {
        (void)currentMicros;
        player->copy();
        if (_repeat && !player->isActive()) {
            DIAG(F("WaveShare::_loop() - Looping track %d/%d"), _currentFolder, _currentTrack);
            playFile(_currentFolder, _currentTrack);
        }
    }

    void _write(VPIN vpin, int value) override {
        (void)vpin;
        if (value) {
            if (_currentTrack) {
                player->play();
            }
        } else {
            _repeat = false;
            _currentTrack = 0;
            player->stop();
        }
    }

    int _read(VPIN vpin) override {
        (void)vpin;
        return player->isActive();
    }

protected:
    void _writeAnalogue(VPIN vpin, int v1, uint8_t v2 = 0, uint16_t cmd = 0) override {
        (void)vpin;
        //DIAG(F("WaveShare::_writeAnalogue(vpin=%u, v1=%d, cmd=%u, v2=%d)"), (int)vpin, v1, (int)cmd, v2);
        switch (cmd) {
            case DF_REPEATPLAY:
            case DF_PLAY:
                // TODO START PLAYING track v1 and set volume v2
                _repeat = (cmd == DF_REPEATPLAY);
                if (player->isActive()) {
                    player->stop();
                    delay(50);
                }
                if (v2) {
                    player->setVolume(v2 * 0.03f);
                }
                _currentTrack = v1;
                playFile(_currentFolder, _currentTrack);
                break;

            case DF_FOLDER:
                _currentFolder = (uint8_t)v2;
                break;

            case DF_STOPPLAY:
                _repeat = false;
                _currentTrack = 0;
                player->stop();
                break;

            case DF_VOL:
                player->setVolume(v2 * 0.03f);
                break;

            case DF_EQ:
                //Ignore, not supported by this device
                break;

            case DF_PAUSE:
                player->stop();
                break;

            case DF_RESUME:
                player->play();
                break;

            case DF_RESET:
                player->stop();
                _repeat = false;
                _currentFolder = 1;
                _currentTrack = 0;
                break;
        }
    }

private:
    // File lookup table entry
    struct FileEntry {
        FileEntry *next;
        uint8_t dirNum;   // Directory number
        uint8_t trackNum;  // Track number (first 3 digits of filename)
        String fullPath;   // Complete path to file
    };

    FileEntry *firstFileEntry = nullptr;

    AudioSourceSDFAT<SdFat32, File32> *source;
    MP3DecoderHelix *decoder;
    AudioPlayer *player;
    AudioBoardStream *kit;
    float _currentVolume = 0.7f;

    void listSDFiles(const char *dirPath, uint8_t dirNum = 0) {
        File32 dir;
        if (!dir.open(dirPath)) {
            USB_SERIAL.printf("[SD] Failed to open directory: %s\n", dirPath);
            return;
        }
        USB_SERIAL.printf("[SD] Files in %s (Dir %d):\n", dirPath, dirNum);
        File32 entry;
        while (entry.openNext(&dir, O_RDONLY)) {
            char name[64];
            entry.getName(name, sizeof(name));
            if (entry.isDir()) {
                if (dirNum > 0) {
                    USB_SERIAL.printf("[SD]   DIR  %s/ (skipping subdirectory)\n", name);
                    entry.close();
                    continue;
                }
                auto nextDirNum = atoi(name);
                if (nextDirNum == 0) {
                    USB_SERIAL.printf("[SD]   DIR  %s/ (skipping non-numeric directory)\n", name);
                    entry.close();
                    continue;
                }
                USB_SERIAL.printf("[SD]   DIR  %s/\n", name);
                // Recursively list subdirectory with incremented dir number
                char subPath[128];
                snprintf(subPath, sizeof(subPath), "%s%s", dirPath, name);
                listSDFiles(subPath, nextDirNum);
            } else {
                // Extract track number from filename (first 3 digits)
                uint8_t trackNum = 0;
                if (isdigit(name[0]) && isdigit(name[1]) && isdigit(name[2])) {
                    trackNum = (uint8_t)((name[0] - '0') * 100 + (name[1] - '0') * 10 + (name[2] - '0'));
                }
                USB_SERIAL.printf("[SD]   FILE %s (%lu bytes) [Dir:%d Track:%d]\n",
                                  name,
                                  (unsigned long)entry.fileSize(),
                                  dirNum,
                                  trackNum);

                // Add to lookup table
                auto fe = new FileEntry();
                fe->dirNum = dirNum;
                fe->trackNum = trackNum;
                fe->next = firstFileEntry;
                firstFileEntry = fe;
                char fullPath[128];
                snprintf(fullPath, sizeof(fullPath), "%s/%s", dirPath, name);
                fe->fullPath = String(fullPath);
            }
            entry.close();
        }
        dir.close();
    }

    void playFile(const byte folder, const int16_t file) {
        DIAG(F("WaveShare::playFile(folder=%u, file=%d)"), (int)folder, (int)file);

        // Try lookup table
        FileEntry *fe;
        for (fe = firstFileEntry; fe; fe = fe->next) {
            if (fe->dirNum == folder && fe->trackNum == file) {
                fe->fullPath;
                DIAG(F("PLAY_FILE %s (from lookup table)\n"), fe->fullPath.c_str());
                player->setPath(fe->fullPath.c_str());
                player->play();
                return;
            }
        }

        DIAG(F("[SD] File %d/%d not in lookup table\n"), folder, file);
    }
};

#endif
#endif
