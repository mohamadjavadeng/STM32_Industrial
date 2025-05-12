/*
 * modbusMaster.c
 *
 *  Created on: May 12, 2025
 *      Author: Mohamad
 */

#include "modbusMaster.h"
#include "modbus_crc.h"
#include <string.h>


void Modbus_Init(ModbusMaster* modbus, UART_HandleTypeDef* huart, GPIO_TypeDef* EN_Port, uint16_t EN_Pin){
	modbus->huart = huart;
    modbus->EN_Port = EN_Port;
    modbus->EN_Pin = EN_Pin;
    // Set DE pin to output mode and disable transmission initially
	HAL_GPIO_WritePin(modbus->EN_Port, modbus->EN_Pin, GPIO_PIN_RESET);

	// Optional: You could initialize buffers here to zero
	memset(modbus->txBuffer, 0, sizeof(modbus->txBuffer));
	memset(modbus->rxBuffer, 0, sizeof(modbus->rxBuffer));
}


//Send command function
static ModbusStatus Modbus_SendRequest(ModbusMaster* modbus, uint8_t requestLength, uint8_t expectedResponseLength) {
    HAL_GPIO_WritePin(modbus->EN_Port, modbus->EN_Pin, GPIO_PIN_SET); // Enable transmit
    HAL_UART_Transmit(modbus->huart, modbus->txBuffer, requestLength, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(modbus->EN_Port, modbus->EN_Pin, GPIO_PIN_RESET); // Disable transmit

    if (HAL_UART_Receive(modbus->huart, modbus->rxBuffer, expectedResponseLength, 1000) != HAL_OK) {
        return MODBUS_TIMEOUT;
    }

    uint16_t crc = crc16(modbus->rxBuffer, expectedResponseLength - 2);
    uint16_t receivedCRC = modbus->rxBuffer[expectedResponseLength - 2] | (modbus->rxBuffer[expectedResponseLength - 1] << 8);
    if (crc != receivedCRC) {
        return MODBUS_INVALID_CRC;
    }

    return MODBUS_OK;
}


ModbusStatus Modbus_ReadHoldingRegisters(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint16_t* data) {
    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
    modbus->txBuffer[2] = startAddress >> 8;
    modbus->txBuffer[3] = startAddress & 0xFF;
    modbus->txBuffer[4] = quantity >> 8;
    modbus->txBuffer[5] = quantity & 0xFF;

    uint16_t crc = crc16(modbus->txBuffer, 6);
    modbus->txBuffer[6] = crc & 0xFF;
    modbus->txBuffer[7] = crc >> 8;

    uint8_t responseLength = 5 + 2 * quantity; // SlaveID + Function + ByteCount + Data + CRC
    ModbusStatus status = Modbus_SendRequest(modbus, 8, responseLength);
    if (status != MODBUS_OK) return status;

    for (uint16_t i = 0; i < quantity; i++) {
        data[i] = (modbus->rxBuffer[3 + i * 2] << 8) | modbus->rxBuffer[4 + i * 2];
    }

    return MODBUS_OK;
}

ModbusStatus Modbus_WriteSingleRegister(ModbusMaster* modbus, uint8_t slaveID, uint16_t address, uint16_t value) {
    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_WRITE_SINGLE_REGISTER;
    modbus->txBuffer[2] = address >> 8;
    modbus->txBuffer[3] = address & 0xFF;
    modbus->txBuffer[4] = value >> 8;
    modbus->txBuffer[5] = value & 0xFF;

    uint16_t crc = crc16(modbus->txBuffer, 6);
    modbus->txBuffer[6] = crc & 0xFF;
    modbus->txBuffer[7] = crc >> 8;

    return Modbus_SendRequest(modbus, 8, 8); // Response is echo of request
}

ModbusStatus Modbus_WriteSingleCoil(ModbusMaster* modbus, uint8_t slaveID, uint16_t address, uint8_t value) {
    uint16_t coilValue = (value != 0) ? 0xFF00 : 0x0000;

    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_WRITE_SINGLE_COIL;
    modbus->txBuffer[2] = address >> 8;
    modbus->txBuffer[3] = address & 0xFF;
    modbus->txBuffer[4] = coilValue >> 8;
    modbus->txBuffer[5] = coilValue & 0xFF;

    uint16_t crc = crc16(modbus->txBuffer, 6);
    modbus->txBuffer[6] = crc & 0xFF;
    modbus->txBuffer[7] = crc >> 8;

    return Modbus_SendRequest(modbus, 8, 8);
}

ModbusStatus Modbus_ReadCoils(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint8_t* coilStatus) {
    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_READ_COILS;
    modbus->txBuffer[2] = startAddress >> 8;
    modbus->txBuffer[3] = startAddress & 0xFF;
    modbus->txBuffer[4] = quantity >> 8;
    modbus->txBuffer[5] = quantity & 0xFF;

    uint16_t crc = crc16(modbus->txBuffer, 6);
    modbus->txBuffer[6] = crc & 0xFF;
    modbus->txBuffer[7] = crc >> 8;

    uint8_t byteCount = (quantity + 7) / 8;
    ModbusStatus status = Modbus_SendRequest(modbus, 8, 5 + byteCount);
    if (status != MODBUS_OK) return status;

    for (uint8_t i = 0; i < byteCount; i++) {
        coilStatus[i] = modbus->rxBuffer[3 + i];
    }

    return MODBUS_OK;
}

ModbusStatus Modbus_ReadDiscreteInputs(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, uint8_t* inputStatus) {
    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_READ_DISCRETE_INPUTS;
    modbus->txBuffer[2] = startAddress >> 8;
    modbus->txBuffer[3] = startAddress & 0xFF;
    modbus->txBuffer[4] = quantity >> 8;
    modbus->txBuffer[5] = quantity & 0xFF;

    uint16_t crc = crc16(modbus->txBuffer, 6);
    modbus->txBuffer[6] = crc & 0xFF;
    modbus->txBuffer[7] = crc >> 8;

    uint8_t byteCount = (quantity + 7) / 8;
    ModbusStatus status = Modbus_SendRequest(modbus, 8, 5 + byteCount);
    if (status != MODBUS_OK) return status;

    for (uint8_t i = 0; i < byteCount; i++) {
        inputStatus[i] = modbus->rxBuffer[3 + i];
    }

    return MODBUS_OK;
}

ModbusStatus Modbus_WriteMultipleCoils(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, const uint8_t* values) {
    uint8_t byteCount = (quantity+7) / 8;

    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_WRITE_MULTIPLE_COILS;
    modbus->txBuffer[2] = startAddress >> 8;
    modbus->txBuffer[3] = startAddress & 0xFF;
    modbus->txBuffer[4] = quantity >> 8;
    modbus->txBuffer[5] = quantity & 0xFF;
    modbus->txBuffer[6] = byteCount;

    for (uint8_t i = 0; i < byteCount; i++) {
        modbus->txBuffer[7 + i] = values[i];
    }

    uint16_t crc = crc16(modbus->txBuffer, 7 + byteCount);
    modbus->txBuffer[7 + byteCount] = crc & 0xFF;
    modbus->txBuffer[8 + byteCount] = crc >> 8;

    return Modbus_SendRequest(modbus, 9 + byteCount, 8);
}

ModbusStatus Modbus_WriteMultipleRegisters(ModbusMaster* modbus, uint8_t slaveID, uint16_t startAddress, uint16_t quantity, const uint16_t* values) {
    modbus->txBuffer[0] = slaveID;
    modbus->txBuffer[1] = MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS;
    modbus->txBuffer[2] = startAddress >> 8;
    modbus->txBuffer[3] = startAddress & 0xFF;
    modbus->txBuffer[4] = quantity >> 8;
    modbus->txBuffer[5] = quantity & 0xFF;
    modbus->txBuffer[6] = quantity * 2;

    for (uint16_t i = 0; i < quantity; i++) {
        modbus->txBuffer[7 + i * 2] = values[i] >> 8;
        modbus->txBuffer[8 + i * 2] = values[i] & 0xFF;
    }

    uint16_t crc = crc16(modbus->txBuffer, 7 + quantity * 2);
    modbus->txBuffer[7 + quantity * 2] = crc & 0xFF;
    modbus->txBuffer[8 + quantity * 2] = crc >> 8;

    return Modbus_SendRequest(modbus, 9 + quantity * 2, 8);
}
