#ifndef ModbusRTUComm_h
#define ModbusRTUComm_h

#include "Arduino.h"
#include "ModbusADU.h"

enum ModbusRTUCommError : uint8_t {
  MODBUS_RTU_COMM_SUCCESS = 0,
  MODBUS_RTU_COMM_TIMEOUT = 1,
  MODBUS_RTU_COMM_FRAME_ERROR = 2,
  MODBUS_RTU_COMM_CRC_ERROR = 3
};

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, int8_t dePin = -1, int8_t rePin = -1);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);
    void setTimeout(unsigned long timeout);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    void writeAdu(ModbusADU& adu);
    void clearRxBuffer();

  private:
    Stream& _serial;
    int8_t _dePin;
    int8_t _rePin;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    unsigned long _postDelay = 0;
    unsigned long _readTimeout = 0;
};

#endif