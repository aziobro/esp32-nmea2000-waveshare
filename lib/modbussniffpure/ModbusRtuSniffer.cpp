#include "ModbusRtuSniffer.h"
#include "ModbusCrc16.h"
#include <cstring>

ModbusRtuSniffer::ModbusRtuSniffer(uint32_t gapThresholdMs) : gapThresholdMs(gapThresholdMs)
{
}

void ModbusRtuSniffer::feedByte(uint8_t b, uint32_t timestampMs)
{
    if (haveLastByteTime && bufLen > 0 && (timestampMs - lastByteTime) >= gapThresholdMs)
    {
        finalizeCurrentBuffer();
    }

    if (bufLen == 0)
    {
        frameStartTime = timestampMs;
    }

    if (bufLen < MAX_FRAME_LEN)
    {
        buffer[bufLen++] = b;
    }
    else
    {
        // Longer than any real Modbus RTU ADU can be - not a valid frame
        // no matter how it's split. Discard and restart on this byte.
        bufLen = 0;
        frameStartTime = timestampMs;
        buffer[bufLen++] = b;
    }

    lastByteTime = timestampMs;
    haveLastByteTime = true;
}

void ModbusRtuSniffer::poll(uint32_t nowMs)
{
    if (haveLastByteTime && bufLen > 0 && (nowMs - lastByteTime) >= gapThresholdMs)
    {
        finalizeCurrentBuffer();
    }
}

void ModbusRtuSniffer::finalizeCurrentBuffer()
{
    if (bufLen == 0)
        return;

    if (!pendingFrameReady)
    {
        pendingFrame.len = bufLen;
        memcpy(pendingFrame.data, buffer, bufLen);
        pendingFrame.crcValid = ModbusCrc16::validateFrame(buffer, bufLen);
        pendingFrame.timestampMs = frameStartTime;
        pendingFrameReady = true;
    }
    // else: caller hasn't drained the previous frame yet - drop this one
    // rather than overwrite the still-unconsumed one.

    bufLen = 0;
}

ModbusRtuSniffer::Frame ModbusRtuSniffer::takeFrame()
{
    Frame f = pendingFrame;
    pendingFrameReady = false;
    pendingFrame = Frame();
    return f;
}

void ModbusRtuSniffer::reset()
{
    bufLen = 0;
    haveLastByteTime = false;
    lastByteTime = 0;
    frameStartTime = 0;
    pendingFrameReady = false;
    pendingFrame = Frame();
}
