/*
 * modbusMaster.h
 *
 *  Created on: May 12, 2025
 *      Author: Mohamad
 */

#ifndef INC_MODBUSMASTER_H_
#define INC_MODBUSMASTER_H_

#include "main.h"
#include "modbus_crc.h"
#include <stdint.h>

//-------------- define
#define MODBUS_BUFFER_SIZE 256

#define MODBUS_FUNCTION_READ_COILS            0x01
#define MODBUS_FUNCTION_READ_DISCRETE_INPUTS  0x02
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_READ_INPUT_REGISTERS  0x04
#define MODBUS_FUNCTION_WRITE_SINGLE_COIL     0x05
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_FUNCTION_WRITE_MULTIPLE_COILS  0x0F
#define MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS 0x10

#define MODBUS_EXCEPTION_MASK 0x80


// Modbus Master handle
typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;
    uint8_t txBuffer[MODBUS_BUFFER_SIZE];
    uint8_t rxBuffer[MODBUS_BUFFER_SIZE];
} ModbusMaster;

typedef enum {
    MODBUS_OK,
    MODBUS_ERROR,
    MODBUS_TIMEOUT,
    MODBUS_INVALID_CRC,
    MODBUS_INVALID_RESPONSE
} ModbusStatus;

// API
void Modbus_Init(ModbusMaster* modbus, UART_HandleTypeDef* huart, GPIO_TypeDef* EN_Port, uint16_t EN_Pin);
ModbusStatus Modbus_ReadHoldingRegisters(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint16_t* data);
ModbusStatus Modbus_WriteSingleRegister(ModbusMaster* modbus, uint8_t slaveID, uint16_t address, uint16_t value);
ModbusStatus Modbus_WriteSingleCoil(ModbusMaster* modbus, uint8_t slaveID, uint16_t address, uint8_t value);
ModbusStatus Modbus_ReadCoils(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint8_t* coilStatus);
ModbusStatus Modbus_ReadDiscreteInputs(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint8_t* inputStatus);
ModbusStatus Modbus_ReadInputRegisters(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint16_t* data);
ModbusStatus Modbus_WriteMultipleCoils(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, const uint8_t* values);
ModbusStatus Modbus_WriteMultipleRegisters(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, const uint16_t* values);

#endif /* INC_MODBUSMASTER_H_ */
