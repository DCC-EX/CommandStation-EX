#ifndef ModbusRTUMaster_h
#define ModbusRTUMaster_h

#include "Arduino.h"
#include "ModbusADU.h"
#include "ModbusRTUComm.h"

enum ModbusRTUMasterError : uint8_t {
  MODBUS_RTU_MASTER_SUCCESS = 0,
  MODBUS_RTU_MASTER_INVALID_ID = 1,
  MODBUS_RTU_MASTER_INVALID_BUFFER = 2,
  MODBUS_RTU_MASTER_INVALID_QUANTITY = 3,
  MODBUS_RTU_MASTER_RESPONSE_TIMEOUT = 4,
  MODBUS_RTU_MASTER_FRAME_ERROR = 5,
  MODBUS_RTU_MASTER_CRC_ERROR = 6,
  MODBUS_RTU_MASTER_UNKNOWN_COMM_ERROR = 7,
  MODBUS_RTU_MASTER_UNEXPECTED_ID = 8,
  MODBUS_RTU_MASTER_EXCEPTION_RESPONSE = 9,
  MODBUS_RTU_MASTER_UNEXPECTED_FUNCTION_CODE = 10,
  MODBUS_RTU_MASTER_UNEXPECTED_LENGTH = 11,
  MODBUS_RTU_MASTER_UNEXPECTED_BYTE_COUNT = 12,
  MODBUS_RTU_MASTER_UNEXPECTED_ADDRESS = 13,
  MODBUS_RTU_MASTER_UNEXPECTED_VALUE = 14,
  MODBUS_RTU_MASTER_UNEXPECTED_QUANTITY = 15
};

class ModbusRTUMaster {
  public:
    ModbusRTUMaster(Stream& serial, int8_t dePin = -1, int8_t rePin = -1);
    void setTimeout(unsigned long timeout);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);

    ModbusRTUMasterError readCoils(uint8_t id, uint16_t startAddress, bool buf[], uint16_t quantity);
    ModbusRTUMasterError readDiscreteInputs(uint8_t id, uint16_t startAddress, bool buf[], uint16_t quantity);
    ModbusRTUMasterError readHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);
    ModbusRTUMasterError readInputRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);

    ModbusRTUMasterError writeSingleCoil(uint8_t id, uint16_t address, bool value);
    ModbusRTUMasterError writeSingleHoldingRegister(uint8_t id, uint16_t address, uint16_t value);
    ModbusRTUMasterError writeMultipleCoils(uint8_t id, uint16_t startAddress, bool buf[], uint16_t quantity);
    ModbusRTUMasterError writeMultipleHoldingRegisters(uint8_t id, uint16_t startAddress, uint16_t buf[], uint16_t quantity);

    uint8_t getExceptionResponse();

  private:
    ModbusRTUComm _rtuComm;
    uint8_t _exceptionResponse = 0;

    ModbusRTUMasterError _readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, bool buf[], uint16_t quantity);
    ModbusRTUMasterError _readValues(uint8_t id, uint8_t functionCode, uint16_t startAddress, uint16_t buf[], uint16_t quantity);
    ModbusRTUMasterError _writeSingleValue(uint8_t id, uint8_t functionCode, uint16_t address, uint16_t value);

    ModbusRTUMasterError _translateCommError(ModbusRTUCommError commError);

};

#endif
