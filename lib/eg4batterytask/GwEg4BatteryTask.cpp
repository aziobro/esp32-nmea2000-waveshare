#include "GwEg4BatteryTask.h"
#include "GwHardware.h"

#ifndef GWEG4BATTERY_ENABLE

void initEg4Battery(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "no EG4 battery RS485 sniffer support for this board, task not started");
}

#else

#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "ModbusRtuSniffer.h"
#include <stdlib.h>
#include <string.h>

static uint32_t formatToSerialConfig(const String &fmt)
{
    if (fmt == "8E1")
        return SERIAL_8E1;
    if (fmt == "8O1")
        return SERIAL_8O1;
    if (fmt == "8N2")
        return SERIAL_8N2;
    return SERIAL_8N1;
}

// Live status for the "data" endpoint. Plain counters, no lock - written
// only from this task's own loop, read from the web server's task on each
// HTTP request; torn reads on a monotonically-increasing counter are
// self-correcting and not worth a mutex for, same reasoning already
// documented for Icm20948Capture's stats
// (lib/icm20948task/GwIcm20948CaptureTask.h).
class Eg4BatteryWebData
{
public:
    volatile uint32_t bytesSeen = 0;
    volatile uint32_t framesSeen = 0;
    volatile uint32_t framesValid = 0;
    volatile uint32_t lastFrameMs = 0;
    volatile bool haveFrame = false;
    int baud = 0;
    String format;

    void toJson(GwJsonDocument &doc)
    {
        doc["bytesSeen"] = bytesSeen;
        doc["framesSeen"] = framesSeen;
        doc["framesValid"] = framesValid;
        doc["lastFrameAgoMs"] = haveFrame ? (uint32_t)(millis() - lastFrameMs) : -1;
        doc["baud"] = baud;
        doc["format"] = format;
    }
};

// Raw-frame capture buffer, downloadable over HTTP - a single-task
// simplification of the lib/icm20948task/GwIcm20948CaptureTask.h pattern
// (RAM buffer sized at "start", control/status/download endpoints, plain
// volatile counters). That task needed a *separate* writer task (and the
// isInit/never-returning-task lifetime split that came with it) because it
// had to keep up with a continuous ~100Hz IMU sample stream without
// stalling it. This bus's frame rate is far lower and each frame is at
// most a couple hundred bytes, so appending directly from the same loop
// that reads the UART is in no danger of missing bytes - no queue, no
// second task, no lifetime-split needed. registerRequestHandler() is still
// only called from begin(), which is only ever invoked from inside
// runEg4BatteryTask()'s own never-returning loop (see below) - the one
// lifetime rule that still applies.
class Eg4BatteryCapture
{
public:
    void begin(GwApi *api)
    {
        api->registerRequestHandler("capControl", [this](AsyncWebServerRequest *request)
                                     {
            String action = request->hasParam("action") ? request->getParam("action")->value() : "";
            if (action == "start") {
                int maxKB = 128;
                if (request->hasParam("maxKB")) maxKB = request->getParam("maxKB")->value().toInt();
                if (maxKB < 1) maxKB = 1;
                if (maxKB > 512) maxKB = 512;
                int maxDurSec = 300;
                if (request->hasParam("maxDurationSec")) maxDurSec = request->getParam("maxDurationSec")->value().toInt();
                if (maxDurSec < 1) maxDurSec = 1;
                if (maxDurSec > 3600) maxDurSec = 3600;
                startCapture(maxKB, maxDurSec);
            }
            else if (action == "stop") stopCapture();
            else if (action == "clear") clearCapture();
            request->send(200, "application/json", "{\"status\":\"OK\"}"); });

        api->registerRequestHandler("capStatus", [this](AsyncWebServerRequest *request)
                                     {
            GwJsonDocument doc(256);
            doc["active"] = captureActive;
            doc["framesCaptured"] = framesCaptured;
            doc["bufferBytesUsed"] = (uint32_t)captureBufferUsed;
            doc["bufferBytesCapacity"] = (uint32_t)captureBufferCapacity;
            doc["maxDurationSec"] = maxDurationSec;
            doc["captureDurationMs"] = captureActive ? (uint32_t)(millis() - captureStartMs) : 0;
            String out;
            serializeJson(doc, out);
            request->send(200, "application/json", out); });

        api->registerRequestHandler("capDownload", [this](AsyncWebServerRequest *request)
                                     {
            if (!captureBuffer || captureBufferUsed == 0) {
                request->send(200, "text/csv", String(csvHeader()));
                return;
            }
            AsyncWebServerResponse *response = request->beginResponse_P(200, "text/csv", (const uint8_t *)captureBuffer, captureBufferUsed);
            response->addHeader("Content-Disposition", "attachment; filename=eg4_modbus_capture.csv");
            request->send(response); });
    }

    // Called from runEg4BatteryTask()'s own loop right after a frame is
    // pulled out of the sniffer.
    void offerFrame(const ModbusRtuSniffer::Frame &f)
    {
        if (!captureActive)
            return;
        unsigned long elapsedSec = (millis() - captureStartMs) / 1000UL;
        if ((int)elapsedSec >= maxDurationSec)
        {
            stopCapture();
            return;
        }

        char row[16 + 1 + 1 + 1 + 5 + 1 + (ModbusRtuSniffer::MAX_FRAME_LEN * 2) + 2];
        int n = snprintf(row, sizeof(row), "%lu,%s,%u,",
                          (unsigned long)f.timestampMs, f.crcValid ? "1" : "0", (unsigned)f.len);
        for (size_t i = 0; i < f.len && n > 0 && (size_t)n < sizeof(row) - 3; i++)
        {
            n += snprintf(row + n, sizeof(row) - n, "%02X", f.data[i]);
        }
        if (n > 0 && (size_t)n < (int)sizeof(row) - 1)
        {
            row[n++] = '\n';
            row[n] = 0;
        }
        if (appendToBuffer(row))
            framesCaptured++;
    }

private:
    volatile bool captureActive = false;
    volatile int maxDurationSec = 300;
    volatile uint32_t framesCaptured = 0;
    unsigned long captureStartMs = 0;

    char *captureBuffer = nullptr;
    size_t captureBufferCapacity = 0;
    size_t captureBufferUsed = 0;

    static const char *csvHeader() { return "timestampMs,crcValid,lenBytes,hex\n"; }

    void startCapture(int maxKB, int maxDurSec)
    {
        clearCapture();
        captureBufferCapacity = (size_t)maxKB * 1024;
        captureBuffer = (char *)malloc(captureBufferCapacity);
        if (!captureBuffer)
        {
            captureBufferCapacity = 0;
            return;
        }
        captureBufferUsed = 0;
        maxDurationSec = maxDurSec;
        captureStartMs = millis();
        captureActive = true;
        appendToBuffer(csvHeader());
    }
    void stopCapture() { captureActive = false; }
    void clearCapture()
    {
        captureActive = false;
        if (captureBuffer)
        {
            free(captureBuffer);
            captureBuffer = nullptr;
        }
        captureBufferCapacity = 0;
        captureBufferUsed = 0;
        framesCaptured = 0;
    }
    bool appendToBuffer(const char *row)
    {
        if (!captureBuffer)
            return false;
        size_t rowLen = strlen(row);
        if (captureBufferUsed + rowLen + 1 > captureBufferCapacity)
        {
            stopCapture(); // full - fail safely rather than overflow or block
            return false;
        }
        memcpy(captureBuffer + captureBufferUsed, row, rowLen);
        captureBufferUsed += rowLen;
        return true;
    }
};

static void runEg4BatteryTask(GwApi *api)
{
    GwLog *logger = api->getLogger();
    GwConfigHandler *config = api->getConfig();

    // Receiver permanently enabled, transmit side never touched again -
    // there is no code path anywhere in this task that writes to Serial1,
    // so this pin setting is the only thing standing between "passive
    // listener" and "bus participant", and it's set once, here, for good.
    pinMode(GWEG4BATTERY_ENA_PIN, OUTPUT);
    digitalWrite(GWEG4BATTERY_ENA_PIN, LOW);

    Eg4BatteryWebData webData;
    Eg4BatteryCapture capture;

    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                 {
        GwJsonDocument doc(200);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });
    // Safe to call from here (not from initEg4Battery): this task never
    // returns, so the api instance registerRequestHandler stores these
    // lambdas against stays alive for the lifetime of the device - see
    // GwEg4BatteryTask.h and the equivalent note in
    // lib/icm20948task/GwIcm20948CaptureTask.h for the discovered-the-hard-
    // way lifetime rule this follows.
    capture.begin(api);

    int baud = 9600;
    config->getValue(baud, GwConfigDefinitions::eg4Baud, 9600);
    String format = "8N1";
    config->getValue(format, GwConfigDefinitions::eg4Format, String("8N1"));
    int gapMs = 6;
    config->getValue(gapMs, GwConfigDefinitions::eg4GapMs, 6);

    Serial1.begin(baud, formatToSerialConfig(format), GWEG4BATTERY_RX_PIN, GWEG4BATTERY_TX_PIN);
    webData.baud = baud;
    webData.format = format;
    LOG_DEBUG(GwLog::LOG, "EG4 battery RS485 sniffer starting: %d baud, %s, gap=%dms, receive-only",
              baud, format.c_str(), gapMs);

    ModbusRtuSniffer sniffer((uint32_t)gapMs);
    unsigned long lastConfigCheckMs = 0;

    while (true)
    {
        bool enabled = true;
        config->getValue(enabled, GwConfigDefinitions::eg4Enable, true);
        if (!enabled)
        {
            delay(200);
            continue;
        }

        unsigned long nowMs = millis();

        // Re-check link config about once a second (not every loop - config
        // reads aren't free) and reopen the UART if the user changed
        // baud/format from the web UI, so trying alternate settings while
        // reverse-engineering the bus doesn't require a reflash each time.
        if (nowMs - lastConfigCheckMs >= 1000)
        {
            lastConfigCheckMs = nowMs;
            int newBaud = baud;
            config->getValue(newBaud, GwConfigDefinitions::eg4Baud, 9600);
            String newFormat = format;
            config->getValue(newFormat, GwConfigDefinitions::eg4Format, String("8N1"));
            int newGapMs = gapMs;
            config->getValue(newGapMs, GwConfigDefinitions::eg4GapMs, 6);

            if (newBaud != baud || newFormat != format)
            {
                baud = newBaud;
                format = newFormat;
                LOG_DEBUG(GwLog::LOG, "EG4 battery sniffer: link config changed, reopening UART1 at %d baud, %s",
                          baud, format.c_str());
                Serial1.end();
                Serial1.begin(baud, formatToSerialConfig(format), GWEG4BATTERY_RX_PIN, GWEG4BATTERY_TX_PIN);
                webData.baud = baud;
                webData.format = format;
                sniffer.reset();
            }
            if (newGapMs != gapMs)
            {
                gapMs = newGapMs;
                sniffer.setGapThresholdMs((uint32_t)gapMs);
            }
        }

        uint32_t drained = 0;
        while (Serial1.available())
        {
            uint8_t b = (uint8_t)Serial1.read();
            sniffer.feedByte(b, nowMs);
            drained++;
        }
        if (drained > 0)
            webData.bytesSeen += drained;

        sniffer.poll(nowMs);
        if (sniffer.hasFrame())
        {
            ModbusRtuSniffer::Frame f = sniffer.takeFrame();
            webData.framesSeen++;
            if (f.crcValid)
                webData.framesValid++;
            webData.lastFrameMs = nowMs;
            webData.haveFrame = true;
            capture.offerFrame(f);
        }

        // Tight but yielding poll loop - short enough (relative to the
        // configured gap threshold, default 6ms) that a single real Modbus
        // frame is never at risk of straddling two poll ticks and getting
        // mistaken for two frames; long enough to actually yield to the
        // scheduler every iteration rather than busy-spinning.
        delay(2);
    }
}

void initEg4Battery(GwApi *api)
{
    api->addCapability("eg4battery", "true");
    api->addUserTask(runEg4BatteryTask, String("eg4BatteryTask"), 4000);
}

#endif
