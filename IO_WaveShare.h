
#ifndef IO_WAVESHARE_H
#define IO_WAVESHARE_H
#if __has_include("AudioTools.h")
#include <Arduino.h>
#include "defines.h"
#include "IODevice.h"
#include <Preferences.h>
#define TCA9555 TCA9555_AUDIOTOOLS
#include <AudioTools.h>
#include <AudioTools/AudioCodecs/CodecMP3Helix.h>
#include <AudioTools/AudioLibs/AudioBoardStream.h>
#include <AudioTools/Disk/AudioSourceSDFAT.h>
#include <AudioBoards/ESP32S3AISmartSpeaker.h>
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
    DIAG(F("WaveShare::create(%u)"), (int)firstVpin);
    if (checkNoOverlap(firstVpin, 1)) {
        new WaveShare(firstVpin);
       }    
   }

  // CAUTION, these constants are copied from FFPlayerBase, and must be kept in sync with that class.
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
    bool _flagLoop = false;        
    uint8_t _currentFolder = 1;    

    
    WaveShare(VPIN firstVpin): IODevice(firstVpin, 1) {
         DIAG(F("WaveShare::constructor(%u)"), (int)firstVpin);
         source = new AudioSourceSDFAT<SdFat32, File32>("", "mp3", -1, 2);
         decoder = new MP3DecoderHelix();
         kit = new AudioBoardStream(ESP32S3AISmartSpeaker);
         player = new AudioPlayer(*source, *kit, *decoder);
         addDevice(this);
    } 

public:
    void _begin() override { 
        DIAG(F("WaveShare::_begin()"));
        
        // TODO initialise the WaveShare device here
        currentVolume = 0.7f;
        currentFolderPath = "/";
        // Setup audio output
        auto cfg = kit->defaultConfig(TX_MODE);
        cfg.sd_active = true;
        bool boardReady = kit->begin(cfg);
        USB_SERIAL.printf("Audio board begin: %s\n", boardReady ? "OK" : "FAIL");
        USB_SERIAL.println("Initializing SD card at 2 MHz...");
        
        // Setup player callbacks (before SD card)
        player->begin();
        player->setVolume(0.7f);
        player->setAutoNext(false);
        player->stop();
    
    if (!source->begin()) {
        USB_SERIAL.println("ERROR: Failed to initialize SD card!");
        USB_SERIAL.println("Check that SD card is inserted and formatted FAT32");
        USB_SERIAL.printf("SDFAT error code: 0x%02X data: 0x%02X\n",
                          source->getAudioFs().sdErrorCode(),
                          source->getAudioFs().sdErrorData());
    }
    
    USB_SERIAL.println("SD card initialized successfully");
     USB_SERIAL.print("<* \n");
    listSDFiles("/");
     USB_SERIAL.print(" *>\n");
    
    _display();
    }

    void _display() override {
        USB_SERIAL.printf("<* WaveShare Audio Device Status:\n");
        USB_SERIAL.printf("  Current Volume: %.2f\n", currentVolume);
        USB_SERIAL.printf("  Current Folder: %s\n", currentFolderPath.c_str());
        USB_SERIAL.printf("  Is Playing: %s *>\n", player->isActive() ? "Yes" : "No");
    }

    void _loop(unsigned long currentMicros) override {
        (void)currentMicros;
        player->copy();
    }

    
    void _write(VPIN vpin, int value) override {
        (void)vpin;
        if (value) { 
            player->play();
        } else { 
            player->stop();
        }
    }

    int _read(VPIN vpin) override {
         (void)vpin; 
         return player->isActive(); 
        }

protected:
  void _writeAnalogue(VPIN vpin, int v1, uint8_t v2=0, uint16_t cmd=0) override {
        (void)vpin;
        DIAG(F("WaveShare::_writeAnalogue(vpin=%u, v1=%d, cmd=%u, v2=%d)"), (int)vpin, v1, (int)cmd, v2);
        switch (cmd){
            case DF_REPEATPLAY:
            case DF_PLAY:
                // TODO START PLAYING track v1 and set volume v2 
                 //IODevice::write(103,1,false); // Enable SD card
                 //IODevice::write(108,1,false); // Enable audio
         
                _flagLoop = (cmd == DF_REPEATPLAY);
                if (player->isActive()) {
                    player->stop();
                    delay(50);
                }
                if (v2) player->setVolume(v2 * 0.03f);
                playFile(_currentFolder, v1);
                break;
    
            case DF_FOLDER:
                 _currentFolder = (uint8_t)v2; 
                // TODO queuePacket(DF_FOLDER, _currentFolder, _lastTrack);
                break;
            case DF_STOPPLAY:
                _flagLoop = false;
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
                break;
        }
    }  

    private:
    // File lookup table entry
    struct FileEntry {
        FileEntry * next;
        uint8_t dirNum;      // Directory number
        uint8_t trackNum;    // Track number (first 3 digits of filename)
        String fullPath;     // Complete path to file
    };

    FileEntry * firstFileEntry=nullptr;
    
    AudioSourceSDFAT<SdFat32, File32> * source;
    MP3DecoderHelix * decoder;
    AudioPlayer * player;
    AudioBoardStream * kit;
    float currentVolume = 0.7f;
    String currentFolderPath = "/";

   
     void listSDFiles(const char* dirPath, uint8_t dirNum = 0) {
       
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
                if (dirNum>0) {
                    USB_SERIAL.printf("[SD]   DIR  %s/ (skipping subdirectory)\n", name);
                    entry.close();
                    continue;
                }
                auto nextDirNum=atoi(name);
                if (nextDirNum==0) {
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
                                  name, (unsigned long)entry.fileSize(), dirNum, trackNum);
                
                // Add to lookup table
                auto fe=new FileEntry();
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
    FileEntry * fe;
    for (fe = firstFileEntry; fe; fe=fe->next) {
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
