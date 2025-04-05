
// sensorCAM parser.cpp version 3.03  Sep 2024
#include "CamParser.h"
#include "FSH.h"
#include "IO_EXSensorCAM.h"

#ifndef SENSORCAM_VPIN  // define CAM vpin (700?) in config.h
#define SENSORCAM_VPIN 0
#endif
#define CAM_VPIN SENSORCAM_VPIN
#ifndef SENSORCAM2_VPIN
#define SENSORCAM2_VPIN CAM_VPIN
#endif
#ifndef SENSORCAM3_VPIN
#define SENSORCAM3_VPIN 0
#endif
const int CAMVPINS[] = {CAM_VPIN, SENSORCAM_VPIN, SENSORCAM2_VPIN, SENSORCAM3_VPIN};
const int16_t ver = 30177;
const int16_t ve = 2899;

VPIN EXSensorCAM::CAMBaseVpin = CAM_VPIN;

bool CamParser::parseN(Print* stream, byte paramCount, int16_t p[]) {
  (void)stream;                          // probably unused parameter
  VPIN vpin = EXSensorCAM::CAMBaseVpin;  // use current CAM selection

  if (paramCount == 0) {
    DIAG(F("vpin:%d EXSensorCAMs defined at Vpins #1@ %d #2@ %d #3@ %d"), vpin, CAMVPINS[1], CAMVPINS[2], CAMVPINS[3]);
    return true;
  }
  uint8_t camop = p[0];  // cam oprerator
  int param1 = 0;
  int16_t param3 = 9999;  // =0 could invoke parameter changes. & -1 gives later errors

  if (camop == 'C') {
    if (p[1] >= 100)
      EXSensorCAM::CAMBaseVpin = p[1];
    if (p[1] < 4)
      EXSensorCAM::CAMBaseVpin = CAMVPINS[p[1]];
    DIAG(F("CAM base Vpin: %c %d "), p[0], EXSensorCAM::CAMBaseVpin);
    return true;
  }
  if (camop < 100) {                  // switch CAM# if p[1] dictates
    if (p[1] >= 100 && p[1] < 400) {  // limits to CAM# 1 to 3 for now
      vpin = CAMVPINS[p[1] / 100];
      EXSensorCAM::CAMBaseVpin = vpin;
      DIAG(F("switching to CAM %d baseVpin:%d"), p[1] / 100, vpin);
      p[1] = p[1] % 100;  // strip off CAM #
    }
  }
  if (EXSensorCAM::CAMBaseVpin == 0)
    return false;  // no cam defined

  // send UPPER case to sensorCAM to flag binary data from a DCCEX-CS parser
  switch (paramCount) {
    case 1:  //<N ver> produces '^'
      if ((p[0] == ve) || (p[0] == ver) || (p[0] == 'V'))
        camop = '^';
      if (STRCHR_P((const char*)F("EFGMQRVW^"), camop) == nullptr)
        return false;
      if (camop == 'Q')
        param3 = 10;  //<NQ> for activation state of all 10 banks of sensors
      if (camop == 'F')
        camop = ']';  //<NF> for Reset/Finish webCAM.
      break;          // F Coded as ']' else conflicts with <Nf %%>

    case 2:  //<N camop p1>
      if (STRCHR_P((const char*)F("ABFILMNOPQRSTUV"), camop) == nullptr)
        return false;
      param1 = p[1];
      break;

    case 3:  //<N vpin rowY colx > or <N cmd p1 p2>
      camop = p[0];
      if (p[0] >= 100) {  // vpin - i.e. NOT 'A' through 'Z'
        if (p[1] > 236 || p[1] < 0)
          return false;  // row
        if (p[2] > 316 || p[2] < 0)
          return false;  // column
        camop = 0x80;    // special 'a' case for IO_SensorCAM
        vpin = p[0];
      } else if (STRCHR_P((const char*)F("IJMNT"), camop) == nullptr)
        return false;
      param1 = p[1];
      param3 = p[2];
      break;

    case 4:  //<N a id row col>
      if (camop != 'A')
        return false;  // must start with 'a'
      if (p[3] > 316 || p[3] < 0)
        return false;
      if (p[2] > 236 || p[2] < 0)
        return false;
      if (p[1] > 97 || p[1] < 0)
        return false;                             // treat as bsNo.
      vpin = vpin + (p[1] / 10) * 8 + p[1] % 10;  // translate p[1]
      camop = 0x80;                               // special 'a' case for IO_SensorCAM
      param1 = p[2];                              // row
      param3 = p[3];                              // col
      break;

    default:
      return false;
  }
  DIAG(F("CamParser: %d %c %d %d"), vpin, camop, param1, param3);
  IODevice::writeAnalogue(vpin, param1, camop, param3);
  return true;
}