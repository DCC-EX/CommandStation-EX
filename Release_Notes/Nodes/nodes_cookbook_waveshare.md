# WaneShare-audio-S2 Support

THis device incorporates a speaker and SD file reader.

It has a supporting IO_WaveShare HAL driver that will only operate or compile on this device. 

The compilation also needs to use the platformio environment `waveshare_esp32_s3_audio` to ensure the cotrrect cpu type and libraries are linked.

## Playing Audio files

By configuring this device with the following in myAutomation.h

```cpp
HAL(WaveShare, 7777)
SHARED_WRITE_VPINS(7777,1)
```

This loads the WaveShare driver on VPIN 7777 and tells the node system to accept incoming manipulations of the VPIN.

This means that ANY node in the system can use the EXRAIL PLAY_* and ```<y``` commands which normally drive a DFPlayer  and this node will operate in the same way.

The node can also handle things like this. 

```cpp
ONSENSOR(7777)
  IF(7777) SCREEN(0,4,"Play started")
  ELSE     SCREEN(0,4,"Play ended")
  ENDIF
DONE
```
