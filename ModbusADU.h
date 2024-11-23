#ifndef ModbusADU_h
#define ModbusADU_h

#include "Arduino.h"

class ModbusADU {
  public:
    uint8_t *rtu = _adu + 6;
    uint8_t *tcp = _adu;
    uint8_t *pdu = _adu + 7;
    uint8_t *data = _adu + 8;

    void setTransactionId(uint16_t transactionId);
    void setProtocolId(uint16_t protocolId);
    void setLength(uint16_t length);
    void setUnitId(uint8_t unitId);
    void setFunctionCode(uint8_t functionCode);
    void setDataRegister(uint8_t index, uint16_t value);

    void setRtuLen(uint16_t rtuLen);
    void setTcpLen(uint16_t tcpLen);
    void setPduLen(uint16_t pduLen);
    void setDataLen(uint16_t dataLen);

    uint16_t getTransactionId();
    uint16_t getProtocolId();
    uint16_t getLength();
    uint8_t getUnitId();
    uint8_t getFunctionCode();
    uint16_t getDataRegister(uint8_t index);

    uint16_t getRtuLen();
    uint16_t getTcpLen();
    uint16_t getPduLen();
    uint16_t getDataLen();

    void updateCrc();
    bool crcGood();

    void prepareExceptionResponse(uint8_t exceptionCode);

  private:
    uint8_t _adu[262];
    void _setRegister(uint8_t *buf, uint16_t index, uint16_t value);
    uint16_t _getRegister(uint8_t *buf, uint16_t index);
    uint16_t _calculateCrc(uint16_t len);
    
};

uint16_t  div8RndUp(uint16_t value);

#endif