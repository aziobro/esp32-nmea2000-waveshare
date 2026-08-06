#pragma once
#include <cstdint>
#include <cstddef>

/*
  Standard Modbus RTU CRC16 (poly 0xA001, init 0xFFFF, no final XOR,
  result transmitted little-endian: low byte first, then high byte).
  Used here purely as a frame-validity check on passively-sniffed bytes,
  not to construct any outgoing frame - this project never transmits on
  this bus.
*/
namespace ModbusCrc16
{
    uint16_t compute(const uint8_t *data, size_t len);

    // True if the last two bytes of [data,len) are this CRC16 (little-endian)
    // of the bytes preceding them. len must be >= 3 (at least one payload
    // byte plus the 2 CRC bytes) or this returns false.
    bool validateFrame(const uint8_t *data, size_t len);
}
