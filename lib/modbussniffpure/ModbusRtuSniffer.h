#pragma once
#include <cstdint>
#include <cstddef>

/*
  Passive Modbus RTU frame sync from a raw byte stream - no delimiter byte
  exists in Modbus RTU, so frame boundaries are inferred purely from
  inter-byte silence gaps (real hardware uses ~3.5 character times; a
  passive sniffer has no real-time bus obligations, so callers are expected
  to pass a generously loose threshold rather than spec-exact timing).
  CRC16 (see ModbusCrc16.h) is the actual correctness gate - a gap can
  mis-split a frame, but a mis-split frame will almost always fail CRC, so
  crcValid on the emitted Frame is what matters, not blind trust in timing.

  Hardware/timer independent - the caller supplies timestamps (any
  monotonically non-decreasing millisecond-ish counter, e.g. millis() on
  the real device or a synthetic counter in tests).
*/
class ModbusRtuSniffer
{
public:
    // Modbus RTU's own max ADU size (spec-defined), also plenty for any
    // realistic battery/inverter register read/write on this bus.
    static const size_t MAX_FRAME_LEN = 256;

    struct Frame
    {
        uint8_t data[MAX_FRAME_LEN];
        size_t len = 0;
        bool crcValid = false;
        // Timestamp of this frame's first byte, same clock as feedByte's.
        uint32_t timestampMs = 0;
    };

    explicit ModbusRtuSniffer(uint32_t gapThresholdMs = 8);

    // Feed one received byte with its arrival timestamp. If the gap since
    // the previous byte is >= gapThresholdMs, whatever was accumulated
    // before this byte is finalized as a completed frame first (check
    // hasFrame() after calling) - this byte then starts the next frame.
    void feedByte(uint8_t b, uint32_t timestampMs);

    // Call periodically (independent of feedByte, e.g. once per task loop
    // iteration) so a frame followed by silence - rather than a new byte -
    // still gets finalized without waiting indefinitely for more traffic.
    void poll(uint32_t nowMs);

    bool hasFrame() const { return pendingFrameReady; }
    // Only valid when hasFrame() is true; clears the pending frame.
    Frame takeFrame();

    void reset();

    // Live reconfiguration (e.g. a web UI setting change) - takes effect
    // on the next gap check, no reset of in-progress buffered bytes needed.
    void setGapThresholdMs(uint32_t ms) { gapThresholdMs = ms; }

private:
    uint8_t buffer[MAX_FRAME_LEN];
    size_t bufLen = 0;
    uint32_t lastByteTime = 0;
    bool haveLastByteTime = false;
    uint32_t frameStartTime = 0;
    uint32_t gapThresholdMs;

    bool pendingFrameReady = false;
    Frame pendingFrame;

    // If a byte buffer is accumulated, validates + copies it into
    // pendingFrame (unless one is already waiting to be taken - in which
    // case, on the assumption the caller is falling behind, the newer
    // frame is dropped rather than the older, still-unconsumed one) and
    // clears the buffer either way.
    void finalizeCurrentBuffer();
};
