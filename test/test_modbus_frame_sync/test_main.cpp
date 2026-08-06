#include <unity.h>
#include "ModbusRtuSniffer.h"
#include "ModbusCrc16.h"
#include <vector>
#include <cstring>

void setUp(void) {}
void tearDown(void) {}

// Builds a valid Modbus RTU frame (payload + correct little-endian CRC16).
static std::vector<uint8_t> makeFrame(std::vector<uint8_t> payload)
{
    uint16_t crc = ModbusCrc16::compute(payload.data(), payload.size());
    payload.push_back((uint8_t)(crc & 0xFF));
    payload.push_back((uint8_t)((crc >> 8) & 0xFF));
    return payload;
}

// Feeds a byte sequence 1ms apart starting at startMs, returns the
// timestamp just after the last byte.
static uint32_t feedBurst(ModbusRtuSniffer &sniffer, const std::vector<uint8_t> &bytes, uint32_t startMs)
{
    uint32_t t = startMs;
    for (uint8_t b : bytes)
    {
        sniffer.feedByte(b, t);
        t += 1;
    }
    return t;
}

void test_clean_frame_finalized_by_poll_after_silence(void)
{
    ModbusRtuSniffer sniffer(8);
    auto frame = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    uint32_t t = feedBurst(sniffer, frame, 0);

    TEST_ASSERT_FALSE(sniffer.hasFrame());
    sniffer.poll(t + 8); // gap has now elapsed with no new byte
    TEST_ASSERT_TRUE(sniffer.hasFrame());

    ModbusRtuSniffer::Frame f = sniffer.takeFrame();
    TEST_ASSERT_EQUAL_UINT32(frame.size(), f.len);
    TEST_ASSERT_TRUE(f.crcValid);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame.data(), f.data, frame.size());
    TEST_ASSERT_FALSE(sniffer.hasFrame()); // consumed
}

void test_poll_before_gap_elapsed_does_not_finalize(void)
{
    ModbusRtuSniffer sniffer(8);
    auto frame = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    uint32_t t = feedBurst(sniffer, frame, 0);

    sniffer.poll(t + 3); // still well within the gap threshold
    TEST_ASSERT_FALSE(sniffer.hasFrame());
}

void test_back_to_back_frames_split_by_gap_on_next_byte(void)
{
    ModbusRtuSniffer sniffer(8);
    auto frame1 = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    auto frame2 = makeFrame({0x01, 0x03, 0x14, 0x11, 0x22});

    uint32_t t = feedBurst(sniffer, frame1, 0);
    TEST_ASSERT_FALSE(sniffer.hasFrame());

    // Big gap, then frame2 arrives - feeding its first byte should finalize
    // frame1 without needing a separate poll() call.
    t += 20;
    feedBurst(sniffer, frame2, t);

    TEST_ASSERT_TRUE(sniffer.hasFrame());
    ModbusRtuSniffer::Frame f1 = sniffer.takeFrame();
    TEST_ASSERT_EQUAL_UINT32(frame1.size(), f1.len);
    TEST_ASSERT_TRUE(f1.crcValid);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame1.data(), f1.data, frame1.size());

    // frame2 is still accumulating (no gap after it yet in this test).
    TEST_ASSERT_FALSE(sniffer.hasFrame());
}

void test_garbage_noise_yields_crc_invalid_frame(void)
{
    ModbusRtuSniffer sniffer(8);
    std::vector<uint8_t> noise = {0xDE, 0xAD, 0xBE, 0xEF};
    uint32_t t = feedBurst(sniffer, noise, 0);
    sniffer.poll(t + 8);

    TEST_ASSERT_TRUE(sniffer.hasFrame());
    ModbusRtuSniffer::Frame f = sniffer.takeFrame();
    TEST_ASSERT_EQUAL_UINT32(noise.size(), f.len);
    TEST_ASSERT_FALSE(f.crcValid);
}

void test_truncated_frame_too_short_for_crc_is_invalid(void)
{
    ModbusRtuSniffer sniffer(8);
    std::vector<uint8_t> tooShort = {0x01, 0x03};
    uint32_t t = feedBurst(sniffer, tooShort, 0);
    sniffer.poll(t + 8);

    TEST_ASSERT_TRUE(sniffer.hasFrame());
    ModbusRtuSniffer::Frame f = sniffer.takeFrame();
    TEST_ASSERT_EQUAL_UINT32(2, f.len);
    TEST_ASSERT_FALSE(f.crcValid);
}

void test_oversized_stream_discards_and_restarts(void)
{
    ModbusRtuSniffer sniffer(8);
    // Feed well beyond MAX_FRAME_LEN with no gaps - must not crash, and
    // must recover to a small trailing buffer rather than wedge.
    uint32_t t = 0;
    for (size_t i = 0; i < ModbusRtuSniffer::MAX_FRAME_LEN + 20; i++)
    {
        sniffer.feedByte((uint8_t)(i & 0xFF), t);
        t += 1;
    }
    sniffer.poll(t + 8);
    TEST_ASSERT_TRUE(sniffer.hasFrame());
    ModbusRtuSniffer::Frame f = sniffer.takeFrame();
    // Only the last (MAX_FRAME_LEN+20) % restart-boundary bytes remain -
    // specifically, the discard-and-restart happens exactly at the byte
    // that would have overflowed, so 20 bytes are left over.
    TEST_ASSERT_EQUAL_UINT32(20, f.len);
}

void test_single_slot_drops_newer_frame_when_unconsumed(void)
{
    ModbusRtuSniffer sniffer(8);
    auto frame1 = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    auto frame2 = makeFrame({0x02, 0x04, 0x00, 0x01});

    uint32_t t = feedBurst(sniffer, frame1, 0);
    sniffer.poll(t + 8); // frame1 now pending, not yet taken

    t += 8;
    t = feedBurst(sniffer, frame2, t);
    sniffer.poll(t + 8); // frame2 completes while frame1 is still unconsumed

    TEST_ASSERT_TRUE(sniffer.hasFrame());
    ModbusRtuSniffer::Frame f = sniffer.takeFrame();
    // The original, still-unconsumed frame1 is preserved; frame2 was dropped.
    TEST_ASSERT_EQUAL_UINT32(frame1.size(), f.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame1.data(), f.data, frame1.size());
    TEST_ASSERT_FALSE(sniffer.hasFrame());
}

void test_set_gap_threshold_takes_effect_immediately(void)
{
    ModbusRtuSniffer sniffer(50); // start loose
    auto frame = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    uint32_t t = feedBurst(sniffer, frame, 0);

    sniffer.poll(t + 8); // well under the 50ms threshold - shouldn't finalize
    TEST_ASSERT_FALSE(sniffer.hasFrame());

    sniffer.setGapThresholdMs(8);
    sniffer.poll(t + 9); // now past the new, tighter threshold
    TEST_ASSERT_TRUE(sniffer.hasFrame());
}

void test_reset_clears_pending_state(void)
{
    ModbusRtuSniffer sniffer(8);
    auto frame = makeFrame({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    uint32_t t = feedBurst(sniffer, frame, 0);
    sniffer.poll(t + 8);
    TEST_ASSERT_TRUE(sniffer.hasFrame());

    sniffer.reset();
    TEST_ASSERT_FALSE(sniffer.hasFrame());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_clean_frame_finalized_by_poll_after_silence);
    RUN_TEST(test_poll_before_gap_elapsed_does_not_finalize);
    RUN_TEST(test_back_to_back_frames_split_by_gap_on_next_byte);
    RUN_TEST(test_garbage_noise_yields_crc_invalid_frame);
    RUN_TEST(test_truncated_frame_too_short_for_crc_is_invalid);
    RUN_TEST(test_oversized_stream_discards_and_restarts);
    RUN_TEST(test_single_slot_drops_newer_frame_when_unconsumed);
    RUN_TEST(test_set_gap_threshold_takes_effect_immediately);
    RUN_TEST(test_reset_clears_pending_state);
    return UNITY_END();
}
