/*
 * esp32msghandler.h
 *
 *  Created on: May 13, 2025
 *      Author: Mohamad
 */

#ifndef INC_ESP32MSGHANDLER_H_
#define INC_ESP32MSGHANDLER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <modbus_crc.h>

// Define variable
#define ESP32MSG_BUFFER_SIZE 64
#define ESP32MSG_START_BYTE  0xAA
#define ESP32MSG_RESPONSE_START_BYTE 0xBB

// Command Types
#define ESP32MSG_CMD_READ    	0x01
#define ESP32MSG_CMD_WRITE   	0x02
#define ESP32MSG_CMD_MULREAD    0x03
#define ESP32MSG_CMD_MULWRITE   0x04

// Register Types
#define REG_TYPE_HOLDING     0x01
#define REG_TYPE_INPUT       0x02
#define REG_TYPE_COIL        0x03
#define REG_TYPE_DISCRETE    0x04

// Status Codes
#define ESP32MSG_STATUS_OK       0x00
#define ESP32MSG_STATUS_ERROR    0x01

void ESP32MsgHandler_Init(UART_HandleTypeDef *huart);
void ESP32MsgHandler_Task(); // Call periodically in main loop

// These must be implemented by the user to connect to actual data source
uint16_t ESP32MsgHandler_ReadRegister(uint8_t type, uint16_t addr);
uint8_t ESP32MsgHandler_MultipleReadRegister(uint8_t type, uint16_t addr, uint8_t quantity, uint8_t *buff);
uint8_t ESP32MsgHandler_WriteRegister(uint8_t type, uint16_t addr, uint16_t value);


#endif /* INC_ESP32MSGHANDLER_H_ */
