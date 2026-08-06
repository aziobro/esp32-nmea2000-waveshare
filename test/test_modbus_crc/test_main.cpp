#include <unity.h>
#include "ModbusCrc16.h"
#include <cstring>

void setUp(void) {}
void tearDown(void) {}

void test_crc_standard_check_value(void)
{
    // The standard CRC-16/MODBUS catalogue "check" vector: CRC16 (poly
    // 0x8005/reflected 0xA001, init 0xFFFF, no xorout) of the ASCII bytes
    // "123456789" is 0x4B37 - the same reference value used to validate
    // any CRC-16/MODBUS implementation, independent of anything specific
    // to this project.
    const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x4B37, ModbusCrc16::compute(data, sizeof(data)));
}

void test_crc_empty(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, ModbusCrc16::compute(nullptr, 0));
}

void test_crc_round_trip_validates(void)
{
    uint8_t frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0, 0};
    uint16_t crc = ModbusCrc16::compute(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);
    TEST_ASSERT_TRUE(ModbusCrc16::validateFrame(frame, sizeof(frame)));
}

void test_crc_round_trip_rejects_corrupted_byte(void)
{
    uint8_t frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0, 0};
    uint16_t crc = ModbusCrc16::compute(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);
    frame[2] ^= 0x01; // flip one bit in the payload after CRC was computed
    TEST_ASSERT_FALSE(ModbusCrc16::validateFrame(frame, sizeof(frame)));
}

void test_crc_validate_frame_too_short(void)
{
    uint8_t frame[2] = {0x01, 0x02};
    TEST_ASSERT_FALSE(ModbusCrc16::validateFrame(frame, sizeof(frame)));
    TEST_ASSERT_FALSE(ModbusCrc16::validateFrame(frame, 0));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_standard_check_value);
    RUN_TEST(test_crc_empty);
    RUN_TEST(test_crc_round_trip_validates);
    RUN_TEST(test_crc_round_trip_rejects_corrupted_byte);
    RUN_TEST(test_crc_validate_frame_too_short);
    return UNITY_END();
}
