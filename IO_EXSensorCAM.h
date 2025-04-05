/*  2024/08/14
 *  © 2024, Barry Daniel ESP32-CAM revision
 *
 *  This file is part of EX-CommandStation
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
#define driverVer 305
// v305 less debug & alpha ordered switch
// v304 static oldb0;  t(##[,%%];
// v303 zipped with CS 5.2.76 and uploaded to repo (with debug)
// v302 SEND=StringFormatter::send, remove Sp(), add 'q', memcpy( .8) -> .7);
// v301 improved 'f','p'&'q' code and driver version calc. Correct bsNo calc. for 'a'
// v300 stripped & revised without expander functionality. Needs sensorCAM.h v300 AND CamParser.cpp
// v222 uses '@'for EXIORDD read.  handles <NB $> and <NN $ ##>
// v216 includes 'j' command and uses CamParser rather than myFilter.h Incompatible with v203 senorCAM
// v203 added pvtThreshold to 'i' output
// v201 deleted code for compatibility with CAM pre v171. Needs CAM ver201 with o06 only
// v200 rewrite reduces need for double reads of ESP32 slave CAM. Deleted ESP32CAP.
//  Inompatible with pre-v170 sensorCAM, unless set S06 to 0 and S07 to 1 (o06 & l07 say)
/*
 * The IO_EXSensorCAM.h device driver can integrate with the sensorCAM device.
 * It is modelled on the IO_EXIOExpander.h device driver to include specific needs of the ESP32 sensorCAM
 * This device driver will configure the device on startup, along with CamParser.cpp
 * interacting with the sensorCAM device for all input/output duties.
 *
 * #include "CamParser.h"      in DCCEXParser.cpp
 * #include "IO_EXSensorCAM.h" in IODevice.h
 * To create EX-SensorCAM devices, define them in myHal.cpp: with
 * EXSensorCAM::create(baseVpin,num_vpins,i2c_address)  or
 * alternatively use HAL(EXSensorCAM baseVpin numpins i2c_address) in myAutomation.h
 * also #define SENSORCAM_VPIN baseVpin in config.h
 *
 * void halSetup() {
 *   // EXSensorCAM::create(vpin, num_vpins, i2c_address);
 *   EXSensorCAM::create(700, 80, 0x11);
 * }
 *
 * I2C packet size of 32 bytes (in the Wire library).
 */
#define DIGITALREFRESH 20000UL  // min uSec delay between digital reads of digitalInputStates
#ifndef IO_EX_EXSENSORCAM_H
#define IO_EX_EXSENSORCAM_H
#define SEND StringFormatter::send
#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"
#include "CamParser.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-SensorCAM.
 */
class EXSensorCAM : public IODevice {
 public:
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress))
      new EXSensorCAM(vpin, nPins, i2cAddress);
  }

  static VPIN CAMBaseVpin;

 private:
  // Constructor
  EXSensorCAM(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    // Number of pins cannot exceed 255 (1 byte) because of I2C message structure.
    if (nPins > 80)
      nPins = 80;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }
  //*************************
  void _begin() {
    uint8_t status;
    // Initialise EX-SensorCAM device
    I2CManager.begin();
    if (!I2CManager.exists(_I2CAddress)) {
      DIAG(F("EX-SensorCAM I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
      return;
    } else {
      uint8_t commandBuffer[4] = {EXIOINIT, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8)};
      status = I2CManager.read(_I2CAddress, _inputBuf, sizeof(_inputBuf), commandBuffer, sizeof(commandBuffer));
      // EXIOINIT needed to trigger and send firstVpin to CAM

      if (status == I2C_STATUS_OK) {
        // Attempt to get version, non-blocking results in poor placement of response.  Can be blocking here!
        commandBuffer[0] = '^';  // new version code

        status = I2CManager.read(_I2CAddress, _inputBuf, sizeof(_inputBuf), commandBuffer, 1);
        // for ESP32 CAM, read again for good immediate response version data
        status = I2CManager.read(_I2CAddress, _inputBuf, sizeof(_inputBuf), commandBuffer, 1);

        if (status == I2C_STATUS_OK) {
          _majorVer = _inputBuf[1] / 10;
          _minorVer = _inputBuf[1] % 10;
          _patchVer = _inputBuf[2];
          DIAG(F("EX-SensorCAM device found, I2C:%s, Version v%d.%d.%d"), _I2CAddress.toString(), _majorVer, _minorVer, _patchVer);
        }
      }
      if (status != I2C_STATUS_OK)
        reportError(status);
    }
  }
  //*************************
  // Digital input pin configuration, used to enable on EX-IOExpander device and set pullups if requested.
  // Configuration isn't done frequently so we can use blocking I2C calls here, and so buffers can
  // be allocated from the stack to reduce RAM allocation.
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override {
    (void)configType;
    (void)params;  // unused
    if (_verPrint)
      DIAG(F("_configure() driver IO_EXSensorCAM v0.%d.%d vpin: %d "), driverVer / 100, driverVer % 100, vpin);
    _verPrint = false;  // only give driver versions once
    if (paramCount != 1)
      return false;
    return true;  // at least confirm that CAM is (always) configured (no vpin check!)
  }
  //*************************
  // Analogue input pin configuration, used to enable an EX-IOExpander device.
  int _configureAnalogIn(VPIN vpin) override {
    DIAG(F("_configureAnalogIn() IO_EXSensorCAM vpin %d"), vpin);
    return true;  // NOTE: use of EXRAIL IFGTE() etc use "analog" reads.
  }
  //*************************
  // Main loop, collect both digital and "analog" pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED)
      return;
    // Request block is used for "analogue" (cmd. data) and digital reads from the sensorCAM, which
    // are performed on a cyclic basis.  Writes are performed synchronously as and when requested.
    if (_readState != RDS_IDLE) {  // expecting a return packet
      if (_i2crb.isBusy())
        return;  // If I2C operation still in progress, return
      uint8_t status = _i2crb.status;
      if (status == I2C_STATUS_OK) {  // If device request ok, read input data
        // apparently the above checks do not guarantee a good packet! error rate about 1 pkt per 1000
        // there should be a packet in _CAMresponseBuff[32]
        if ((_CAMresponseBuff[0] & 0x60) >= 0x60) {                               // Buff[0] seems to have ascii cmd header (bit6 high) (o06)
          int error = processIncomingPkt(_CAMresponseBuff, _CAMresponseBuff[0]);  // '~' 'i' 'm' 'n' 't' etc
          if (error > 0)
            DIAG(F("CAM packet header(0x%x) not recognised"), _CAMresponseBuff[0]);
        } else {  // Header not valid - typically replaced by bank 0 data!  To avoid any bad responses set S06 to 0
                  // Versions of sensorCAM.h after v300 should return header for '@' of '`'(0x60) (not 0xE6)
                  // followed by digitalInputStates sensor state array
        }
      } else
        reportError(status, false);  // report i2c eror but don't go offline.
      _readState = RDS_IDLE;
    }

    // If we're not doing anything now, check to see if a new state table transfer, or for 't' repeat, is due.
    if (_readState == RDS_IDLE) {  // check if time for digitalRefresh
      if (currentMicros - _lastDigitalRead > _digitalRefresh) {
        // Issue new read request for digital states.

        _readCommandBuffer[0] = '@';  // start new read of digitalInputStates Table     // non-blocking read
        I2CManager.read(_I2CAddress, _CAMresponseBuff, 32, _readCommandBuffer, 1, &_i2crb);
        _lastDigitalRead = currentMicros;
        _readState = RDS_DIGITAL;

      } else {                                                 // slip in a repeat <NT n> if pending
        if (currentMicros - _lasttStateRead > _tStateRefresh)  // Delay for "analog" command repetitions
          if (_savedCmd[2] > 1) {                              // repeat a 't' command
            for (int i = 0; i < 7; i++) _readCommandBuffer[i] = _savedCmd[i];
            int errors = ioESP32(_I2CAddress, _CAMresponseBuff, 32, _readCommandBuffer, 7);
            _lasttStateRead = currentMicros;
            _savedCmd[2] -= 1;  // decrement repeats
            if (errors == 0)
              return;
            DIAG(F("ioESP32 error %d header 0x%x"), errors, _CAMresponseBuff[0]);
            _readState = RDS_TSTATE;  // this should stop further cmd requests until packet read (or timeout)
          }
      }  // end repeat 't'
    }
  }
  //*************************
  // Obtain the bank of 8 sensors as an "analog" value
  // can be used to track the position through a sequential sensor bank
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED)
      return 0;
    return _digitalInputStates[(vpin - _firstVpin) / 8];
  }
  //*************************
  // Obtain the correct digital sensor input value
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED)
      return 0;
    int pin = vpin - _firstVpin;
    return bitRead(_digitalInputStates[pin / 8], pin % 8);
  }
  //*************************
  // Write digital value.
  void _write(VPIN vpin, int value) override {
    DIAG(F("**_write() vpin %d = %d"), vpin, value);
    return;
  }
  //*************************
  // i2cAddr of ESP32 CAM
  // rBuf   buffer for return packet
  // inbytes number of bytes to request from CAM
  // outBuff holds outbytes to be sent to CAM
  int ioESP32(uint8_t i2cAddr, uint8_t* rBuf, int inbytes, uint8_t* outBuff, int outbytes) {
    uint8_t status = _i2crb.status;

    while (_i2crb.status != I2C_STATUS_OK) {
      status = _i2crb.status;
    }  // wait until bus free

    status = I2CManager.read(i2cAddr, rBuf, inbytes, outBuff, outbytes);

    if (status != I2C_STATUS_OK) {
      DIAG(F("EX-SensorCAM I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
      reportError(status);
      return status;
    }
    return 0;  // 0 for no error != 0 for error number.
  }
  //*************************
  // function to interpret packet from sensorCAM.ino
  // i2cAddr to identify CAM# (if # >1)
  // rBuf contains packet of up to 32 bytes usually with (ascii) cmd header in rBuf[0]
  // sensorCmd command header byte from CAM (in rBuf[0]?)
  int processIncomingPkt(uint8_t* rBuf, uint8_t sensorCmd) {
    // static uint8_t oldb0;   //for debug only
    int k;
    int b;
    char str[] = "11111111";
    // if (sensorCmd <= '~') DIAG(F("processIncomingPkt %c %d %d %d"),rBuf[0],rBuf[1],rBuf[2],rBuf[3]);
    switch (sensorCmd) {
      case '`':  // response to request for digitalInputStates[] table  '@'=>'`'
        memcpy(_digitalInputStates, rBuf + 1, digitalBytesNeeded);
        //      if ( _digitalInputStates[0]!=oldb0) { oldb0=_digitalInputStates[0];  //debug
        //        for (k=0;k<5;k++) {Serial.print(" ");Serial.print(_digitalInputStates[k],HEX);}
        //      }
        break;

      case EXIORDY:  // some commands give back acknowledgement only
        break;

      case CAMERR:  // cmd format error code from CAM
        DIAG(F("CAM cmd error 0xFE 0x%x"), rBuf[1]);
        break;

      case '~':  // information from '^' version request <N v[er]>
        DIAG(F("EX-SensorCAM device found, I2C:%s,CAM Version v%d.%d.%d vpins %u-%u"), _I2CAddress.toString(), rBuf[1] / 10, rBuf[1] % 10, rBuf[2],
             (int)_firstVpin, (int)_firstVpin + _nPins - 1);
        DIAG(F("IO_EXSensorCAM driver  v0.%d.%d vpin: %d "), driverVer / 100, driverVer % 100, _firstVpin);
        break;

      case 'f':
        DIAG(F("(f %%%%) frame header 'f' for bsNo %d/%d - showing Quarter sample (1 row) only"), rBuf[1] / 8, rBuf[1] % 8);
        SEND(&USB_SERIAL, F("<n  row: %d  Ref bytes: "), rBuf[2]);
        for (k = 3; k < 15; k++) SEND(&USB_SERIAL, F("%x%x%s"), rBuf[k] >> 4, rBuf[k] & 15, k % 3 == 2 ? "  " : " ");
        Serial.print(" latest grab: ");
        for (k = 16; k < 28; k++) SEND(&USB_SERIAL, F("%x%x%s"), rBuf[k] >> 4, rBuf[k] & 15, (k % 3 == 0) ? "  " : " ");
        Serial.print(" n>\n");
        break;

      case 'i':  // information from i%%
        k = 256 * rBuf[5] + rBuf[4];
        DIAG(F("(i%%%%[,$$]) Info: Sensor 0%o(%d) enabled:%d status:%d row=%d x=%d Twin=0%o pvtThreshold=%d A~%d"), rBuf[1], rBuf[1], rBuf[3],
             rBuf[2], rBuf[6], k, rBuf[7], rBuf[9], int(rBuf[8]) * 16);
        break;

      case 'm':
        DIAG(F("(m$[,##]) Min/max: $ frames min2flip (trip) %d, maxSensors 0%o, minSensors 0%o, nLED %d,"
               " threshold %d, TWOIMAGE_MAXBS 0%o"),
             rBuf[1], rBuf[3], rBuf[2], rBuf[4], rBuf[5], rBuf[6]);
        break;

      case 'n':
        DIAG(F("(n$[,##]) Nominate: $ nLED %d, ## minSensors 0%o (maxSensors 0%o threshold %d)"), rBuf[4], rBuf[2], rBuf[3], rBuf[5]);
        break;

      case 'p':
        b = rBuf[1] - 2;
        if (b < 4) {
          Serial.print("<n (p%%) Bank empty  n>\n");
          break;
        }
        SEND(&USB_SERIAL, F("<n (p%%) Bank: %d "), (0x7F & rBuf[2]) / 8);
        for (int j = 2; j < b; j += 3)
          SEND(&USB_SERIAL, F(" S[%d%d]: r=%d x=%d"), 0x7F & rBuf[j] / 8, 0x7F & rBuf[j] % 8, rBuf[j + 1], rBuf[j + 2] + 2 * (rBuf[j] & 0x80));
        Serial.print("  n>\n");
        break;

      case 'q':
        for (int i = 0; i < 8; i++) str[i] = ((rBuf[2] << i) & 0x80 ? '1' : '0');
        DIAG(F("(q $) Query bank %c ENABLED sensors(S%c7-%c0): %s "), rBuf[1], rBuf[1], rBuf[1], str);
        break;

      case 't':  // threshold etc. from t##           //bad pkt if 't' FF's
        if (rBuf[1] == 0xFF) {
          Serial.println("<n bad CAM 't' packet: 74 FF  n>");
          _savedCmd[2] += 1;
          return 0;
        }
        SEND(&USB_SERIAL, F("<n (t[##[,%%%%]]) Threshold:%d sensor S00:-%d"), rBuf[1], min(rBuf[2] & 0x7F, 99));
        if (rBuf[2] > 127)
          Serial.print("##* ");
        else {
          if (rBuf[2] > rBuf[1])
            Serial.print("-?* ");
          else
            Serial.print("--* ");
        }
        for (int i = 3; i < 31; i += 2) {
          uint8_t valu = rBuf[i];  // get bsn
          if (valu == 80)
            break;  // 80 = end flag
          else {
            SEND(&USB_SERIAL, F("%d%d:"), (valu & 0x7F) / 8, (valu & 0x7F) % 8);
            if (valu >= 128)
              Serial.print("?-");
            else {
              if (rBuf[i + 1] >= 128)
                Serial.print("oo");
              else
                Serial.print("--");
            }
            valu = rBuf[i + 1];
            SEND(&USB_SERIAL, F("%d%s"), min(valu & 0x7F, 99), (valu < 128) ? "--* " : "##* ");
          }
        }
        Serial.print(" >\n");
        break;

      default:  // header not a recognised cmd character
        DIAG(F("CAM packet header not valid (0x%x) (0x%x) (0x%x)"), rBuf[0], rBuf[1], rBuf[2]);
        return 1;
    }
    return 0;
  }
  //*************************
  // Write (analogue) 8bit (command) values.  Write the parameters to the sensorCAM
  void _writeAnalogue(VPIN vpin, int param1, uint8_t camop, uint16_t param3) override {
    uint8_t outputBuffer[7];
    int errors = 0;
    outputBuffer[0] = camop;
    int pin = vpin - _firstVpin;

    if (camop >= 0x80) {  // case "a" (4p) also (3p) e.g. <N 713 210 310>
      camop = param1;     // put row (0-236) in expected place
      param1 = param3;    // put column in expected place
      outputBuffer[0] = 'A';
      pin = (pin / 8) * 10 + pin % 8;  // restore bsNo. as integer
    }
    if (_deviceState == DEVSTATE_FAILED)
      return;

    outputBuffer[1] = pin;  // vpin => bsn
    outputBuffer[2] = param1 & 0xFF;
    outputBuffer[3] = param1 >> 8;
    outputBuffer[4] = camop;  // command code
    outputBuffer[5] = param3 & 0xFF;
    outputBuffer[6] = param3 >> 8;

    int count = param1 + 1;
    if (camop == 'Q') {
      if (param3 <= 10) {
        count = param3;
        camop = 'B';
      }
      // if(param1<10) outputBuffer[2] = param1*10;
    }
    if (camop == 'B') {  // then 'b'(b%) cmd - can totally deal with that here. (but can't do b%,# (brightSF))
      if (param1 > 97)
        return;
      if (param1 > 9)
        param1 = param1 / 10;  // accept a bsNo
      for (int bnk = param1; bnk < count; bnk++) {
        uint8_t b = _digitalInputStates[bnk];
        char str[] = "11111111";
        for (int i = 0; i < 8; i++)
          if (((b << i) & 0x80) == 0)
            str[i] = '0';
        DIAG(F("(b $) Bank: %d activated byte: 0x%x%x (sensors S%d7->%d0) %s"), bnk, b >> 4, b & 15, bnk, bnk, str);
      }
      return;
    }
    if (outputBuffer[4] == 'T') {  // then 't' cmd
      if (param1 < 31) {           // repeated calls if param < 31
        // for (int i=0;i<7;i++) _savedCmd[i]=outputBuffer[i];
        memcpy(_savedCmd, outputBuffer, 7);
      } else
        _savedCmd[2] = 0;  // no repeats if ##>30
    } else
      _savedCmd[2] = 0;  // no repeats unless 't'

    _lasttStateRead = micros();  // don't repeat until _tStateRefresh mSec

    errors = ioESP32(_I2CAddress, _CAMresponseBuff, 32, outputBuffer, 7);  // send to esp32-CAM
    if (errors == 0)
      return;
    else {  //       if (_CAMresponseBuff[0] != EXIORDY)   //can't be sure what is inBuff[0] !
      DIAG(F("ioESP32 i2c error %d header 0x%x"), errors, _CAMresponseBuff[0]);
    }
  }
  //*************************
  // Display device information and status.
  void _display() override {
    DIAG(F("EX-SensorCAM I2C:%s v%d.%d.%d Vpins %u-%u %S"), _I2CAddress.toString(), _majorVer, _minorVer, _patchVer, (int)_firstVpin,
         (int)_firstVpin + _nPins - 1, _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
  //*************************
  // Helper function for error handling
  void reportError(uint8_t status, bool fail = true) {
    DIAG(F("EX-SensorCAM I2C:%s Error:%d (%S)"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
    if (fail)
      _deviceState = DEVSTATE_FAILED;
  }
  //*************************
  uint8_t _numDigitalPins = 80;
  size_t digitalBytesNeeded = 10;
  uint8_t _CAMresponseBuff[34];

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;

  uint8_t _digitalInputStates[10];
  I2CRB _i2crb;
  uint8_t _inputBuf[12];
  byte _outputBuffer[8];

  bool _verPrint = true;

  uint8_t _readCommandBuffer[8];
  uint8_t _savedCmd[8];  // for repeat 't' command
  // uint8_t _digitalPinBytes = 10;  // Size of allocated memory buffer (may be longer than needed)

  enum { RDS_IDLE, RDS_DIGITAL, RDS_TSTATE };  // Read operation states
  uint8_t _readState = RDS_IDLE;
  // uint8_t cmdBuffer[7]={0,0,0,0,0,0,0};
  unsigned long _lastDigitalRead = 0;
  unsigned long _lasttStateRead = 0;
  unsigned long _digitalRefresh = DIGITALREFRESH;  // Delay refreshing digital inputs for 10ms
  const unsigned long _tStateRefresh = 120000UL;   // Delay refreshing repeat "tState" inputs

  enum {
    EXIOINIT = 0xE0,  // Flag to initialise setup procedure
    EXIORDY = 0xE1,   // Flag we have completed setup procedure, also for EX-IO to ACK setup
    CAMERR = 0xFE
  };
};
#endif
