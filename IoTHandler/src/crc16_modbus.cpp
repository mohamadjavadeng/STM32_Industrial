#include "crc16_modbus.h"

uint16_t crc16Modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)*data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

size_t crc16ModbusAppend(uint8_t* data, size_t len) {
    const uint16_t crc = crc16Modbus(data, len);
    data[len] = (uint8_t)(crc & 0xFF);
    data[len + 1] = (uint8_t)(crc >> 8);
    return len + 2;
}

bool crc16ModbusCheck(const uint8_t* frame, size_t totalLen) {
    if (totalLen < 3) return false;
    const size_t bodyLen = totalLen - 2;
    const uint16_t received = (uint16_t)frame[bodyLen] | (uint16_t)((uint16_t)frame[bodyLen + 1] << 8);
    return crc16Modbus(frame, bodyLen) == received;
}

bool crc16ModbusSelfTest() {
    static const uint8_t kVector[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    return crc16Modbus(kVector, sizeof(kVector)) == 0x4B37;
}
