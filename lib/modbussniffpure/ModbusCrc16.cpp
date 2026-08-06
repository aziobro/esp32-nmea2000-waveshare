#include "ModbusCrc16.h"

uint16_t ModbusCrc16::compute(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (int bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool ModbusCrc16::validateFrame(const uint8_t *data, size_t len)
{
    if (len < 3)
        return false;
    uint16_t computed = compute(data, len - 2);
    uint16_t received = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    return computed == received;
}
