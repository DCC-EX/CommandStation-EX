
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
         source = new AudioSourceSDFAT<SdFat32, File32>("", "mp3", -1);
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
        kit->begin(cfg);
        
        // Setup player callbacks (before SD card)
        player->begin();
        player->setVolume(0.7f);
        player->setAutoNext(false);
        player->stop();
        USB_SERIAL.println("Initializing SD card...");
    
    if (!source->begin()) {
        USB_SERIAL.println("ERROR: Failed to initialize SD card!");
        USB_SERIAL.println("Check that SD card is inserted and formatted FAT32");
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
  void _writeAnalogue(VPIN vpin, int v1, uint8_t cmd, uint16_t v2=0) override {
        (void)vpin;
        if (_deviceState != DEVSTATE_NORMAL) return;
        switch (cmd){
            case DF_REPEATPLAY:
            case DF_PLAY:
                // TODO START PLAYING track v1 and set volume v2 
                IODevice::write(100,3,false); // Enable audio
         
                _flagLoop = (cmd == DF_REPEATPLAY);
                char filename[32];
                snprintf(filename, sizeof(filename), "%s%03d.mp3", 
                  currentFolderPath.c_str(), v1, "mp3");
    
                if (player->isActive()) {
                    player->stop();
                    delay(50);
                }
    
                player->setPath(filename);
                player->setVolume(v2 * 3.0f);
                player->play(); 
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
                player->setVolume(v2 * 3.0f);
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
    AudioSourceSDFAT<SdFat32, File32> * source;
    MP3DecoderHelix * decoder;
    AudioPlayer * player;
    AudioBoardStream * kit;
    float currentVolume = 0.7f;
    String currentFolderPath = "/";

    void listSDFiles(const char* dirPath) {
       
        File32 dir;
        if (!dir.open(dirPath)) {
            USB_SERIAL.printf("[SD] Failed to open directory: %s\n", dirPath);
            return;
        }
        USB_SERIAL.printf("[SD] Files in %s:\n", dirPath);
        File32 entry;
        while (entry.openNext(&dir, O_RDONLY)) {
            char name[64];
            entry.getName(name, sizeof(name));
            if (entry.isDir()) {
                USB_SERIAL.printf("[SD]   DIR  %s/\n", name);
                listSDFiles(name); // Recursively list subdirectory
            } else {
                USB_SERIAL.printf("[SD]   FILE %s (%lu bytes)\n", name, (unsigned long)entry.fileSize());
            }
            entry.close();
        }
        dir.close();
    }
    
        
};
#endif
#endif
