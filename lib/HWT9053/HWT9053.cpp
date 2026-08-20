#include "HWT9053.h"
#include <cstring>

CJY901::CJY901()
{
    ucDevAddr = 0x50;
    data.validMain = false;
    data.validQuat = false;
    data.validTemp = false;
}

uint16_t CJY901::calculateCRC(unsigned char* buffer, int length) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void CJY901::buildReadCommand(unsigned char addr, unsigned char count, unsigned char* cmdOut) {
    cmdOut[0] = ucDevAddr;
    cmdOut[1] = 0x03;
    cmdOut[2] = 0x00;
    cmdOut[3] = addr;
    cmdOut[4] = 0x00;
    cmdOut[5] = count;
    uint16_t crc = calculateCRC(cmdOut, 6);
    cmdOut[6] = crc & 0xFF;
    cmdOut[7] = (crc >> 8) & 0xFF;
}

int16_t CJY901::decodeInt16(unsigned char* buffer, int index) {
    return (int16_t)((buffer[index] << 8) | buffer[index + 1]);
}

int32_t CJY901::decodeInt32(unsigned char* buffer, int index) {
    // Little endian 32-bit (2x 16-bit registers swapped), like struct.pack("<HH", ...) in python
    uint16_t reg0 = (buffer[index] << 8) | buffer[index + 1];
    uint16_t reg1 = (buffer[index + 2] << 8) | buffer[index + 3];
    return (int32_t)((reg1 << 16) | reg0);
}

void CJY901::parseModbusResponse(unsigned char* buffer, int length) {
    if (length < 5) return; // Minimum modbus response length
    if (buffer[0] != ucDevAddr) return;
    if (buffer[1] != 0x03) return; // Only process read holding registers response
    
    unsigned char byteCount = buffer[2];
    if (length < byteCount + 5) return; // Incomplete packet
    
    // Verify CRC
    uint16_t receivedCRC = buffer[3 + byteCount] | (buffer[4 + byteCount] << 8);
    uint16_t calculatedCRC = calculateCRC(buffer, 3 + byteCount);
    if (receivedCRC != calculatedCRC) return;

    if (byteCount == 30) { // 15 registers * 2 bytes = 30 bytes (Main Block)
        data.accelX = decodeInt16(buffer, 3) / SCALE_ACCEL;
        data.accelY = decodeInt16(buffer, 5) / SCALE_ACCEL;
        data.accelZ = decodeInt16(buffer, 7) / SCALE_ACCEL;
        
        data.gyroX = decodeInt16(buffer, 9) / SCALE_GYRO;
        data.gyroY = decodeInt16(buffer, 11) / SCALE_GYRO;
        data.gyroZ = decodeInt16(buffer, 13) / SCALE_GYRO;
        
        data.magX = decodeInt16(buffer, 15) / SCALE_MAG;
        data.magY = decodeInt16(buffer, 17) / SCALE_MAG;
        data.magZ = decodeInt16(buffer, 19) / SCALE_MAG;
        
        // Angles are 32-bit (2 registers each)
        data.roll = decodeInt32(buffer, 21) / SCALE_ANGLE;
        data.pitch = decodeInt32(buffer, 25) / SCALE_ANGLE;
        data.yaw = decodeInt32(buffer, 29) / SCALE_ANGLE;
        
        data.validMain = true;
    } else if (byteCount == 8) { // 4 registers * 2 bytes = 8 bytes (Quaternion Block)
        data.q0 = decodeInt16(buffer, 3) / SCALE_QUAT;
        data.q1 = decodeInt16(buffer, 5) / SCALE_QUAT;
        data.q2 = decodeInt16(buffer, 7) / SCALE_QUAT;
        data.q3 = decodeInt16(buffer, 9) / SCALE_QUAT;
        
        data.validQuat = true;
    } else if (byteCount == 2) { // 1 register = 2 bytes (Temperature Block)
        data.raw_temperature = decodeInt16(buffer, 3);
        data.temperature = data.raw_temperature / SCALE_TEMP;
        
        data.validTemp = true;
    }
}

CJY901 HWT9053 = CJY901();