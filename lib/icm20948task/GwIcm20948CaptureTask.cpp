#include "GwIcm20948CaptureTask.h"
#include "GwHardware.h"
#include "GwJsonDocument.h"
#include <stdlib.h>
#include <string.h>

// GwUserTaskFunction (see GwApi.h) is a plain function pointer - it can't
// capture `this`, so the writer task's entry point is a free function
// reaching the single Icm20948Capture instance (owned as a local in
// runIcm20948Task, same lifetime pattern as the rest of the pipeline
// objects there) via this file-scope pointer, set in begin().
static Icm20948Capture *g_capture = nullptr;

static void icm20948CaptureWriterTaskEntry(GwApi *api)
{
    if (g_capture)
        g_capture->writerTaskLoop(api);
}

void Icm20948Capture::begin(GwApi *api)
{
    GwLog *logger = api->getLogger();
    g_capture = this;
    queue = xQueueCreate(QUEUE_CAPACITY, sizeof(DiagnosticSample));
    if (!queue)
    {
        LOG_DEBUG(GwLog::ERROR, "icm20948 capture: failed to create queue");
        return;
    }
    api->addUserTask(icm20948CaptureWriterTaskEntry, String("icm20948CapWr"), 4000);

    api->registerRequestHandler("capControl", [this](AsyncWebServerRequest *request)
                                 {
        String action = request->hasParam("action") ? request->getParam("action")->value() : "";
        if (action == "start") startCapture();
        else if (action == "stop") stopCapture();
        else if (action == "clear") clearCapture();
        else if (action == "serialOn") serialEnabled = true;
        else if (action == "serialOff") serialEnabled = false;
        if (request->hasParam("rateHz")) {
            int v = request->getParam("rateHz")->value().toInt();
            if (v >= 1 && v <= 100) rateHz = v;
        }
        if (request->hasParam("maxDurationSec")) {
            int v = request->getParam("maxDurationSec")->value().toInt();
            if (v >= 1 && v <= 3600) maxDurationSec = v;
        }
        if (request->hasParam("maxKB")) {
            int v = request->getParam("maxKB")->value().toInt();
            if (v >= 1 && v <= 512) maxKB = v;
        }
        request->send(200, "application/json", "{\"status\":\"OK\"}"); });

    api->registerRequestHandler("capStatus", [this](AsyncWebServerRequest *request)
                                 {
        GwJsonDocument doc(320);
        doc["active"] = captureActive;
        doc["serialEnabled"] = serialEnabled;
        doc["samplesCaptured"] = samplesOffered;
        doc["samplesWritten"] = samplesWritten;
        doc["samplesDropped"] = samplesDropped;
        doc["queueHighWaterMark"] = queueHighWaterMark;
        doc["captureDurationMs"] = captureActive ? (uint32_t)(millis() - captureStartMs) : 0;
        doc["bufferBytesUsed"] = (uint32_t)captureBufferUsed;
        doc["bufferBytesCapacity"] = (uint32_t)captureBufferCapacity;
        doc["rateHz"] = rateHz;
        doc["maxDurationSec"] = maxDurationSec;
        doc["maxKB"] = maxKB;
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    api->registerRequestHandler("capDownload", [this](AsyncWebServerRequest *request)
                                 {
        if (!captureBuffer || captureBufferUsed == 0) {
            request->send(200, "text/csv", String(ImuDiagnostics::csvHeader()) + "\n");
            return;
        }
        AsyncWebServerResponse *response = request->beginResponse_P(200, "text/csv", (const uint8_t *)captureBuffer, captureBufferUsed);
        response->addHeader("Content-Disposition", "attachment; filename=icm20948_capture.csv");
        request->send(response); });
}

void Icm20948Capture::offerSample(const DiagnosticSample &sample)
{
    if (!queue)
        return;
    if (!captureActive && !serialEnabled)
        return; // nothing wants samples right now - skip the rate-limit bookkeeping too

    int hz = rateHz;
    if (hz < 1)
        hz = 1;
    unsigned long minIntervalMs = 1000UL / (unsigned long)hz;
    unsigned long now = millis();
    if (lastOfferMs != 0 && (now - lastOfferMs) < minIntervalMs)
        return;
    lastOfferMs = now;

    samplesOffered++;
    if (xQueueSend(queue, &sample, 0) != pdTRUE)
    {
        samplesDropped++;
        return;
    }
    UBaseType_t depth = uxQueueMessagesWaiting(queue);
    if (depth > queueHighWaterMark)
        queueHighWaterMark = depth;

    // Auto-stop on duration/size limit - checked here too (not just in
    // the writer task) so a producer that's actively offering samples
    // notices promptly even if the writer is momentarily behind.
    if (captureActive)
    {
        unsigned long elapsedSec = (now - captureStartMs) / 1000UL;
        if ((int)elapsedSec >= maxDurationSec)
            stopCapture();
    }
}

void Icm20948Capture::startCapture()
{
    if (captureActive)
        return;
    clearCapture();
    captureBufferCapacity = (size_t)maxKB * 1024;
    captureBuffer = (char *)malloc(captureBufferCapacity);
    if (!captureBuffer)
    {
        // Fail safely - no buffer, no crash, capture just doesn't start.
        captureBufferCapacity = 0;
        return;
    }
    captureBufferUsed = 0;
    captureStartMs = millis();
    lastOfferMs = 0;
    captureActive = true;
}

void Icm20948Capture::stopCapture()
{
    captureActive = false;
}

void Icm20948Capture::clearCapture()
{
    captureActive = false;
    if (captureBuffer)
    {
        free(captureBuffer);
        captureBuffer = nullptr;
    }
    captureBufferCapacity = 0;
    captureBufferUsed = 0;
    samplesOffered = 0;
    samplesWritten = 0;
    samplesDropped = 0;
    queueHighWaterMark = 0;
}

bool Icm20948Capture::appendToBuffer(const char *row)
{
    if (!captureBuffer)
        return false;
    size_t rowLen = strlen(row);
    // +1 for the newline this appends, +1 for the null terminator
    // capDownload doesn't strictly need but keeps this buffer always
    // safely printable/loggable.
    if (captureBufferUsed + rowLen + 2 > captureBufferCapacity)
    {
        stopCapture(); // full - fail safely, stop rather than overflow or block
        return false;
    }
    memcpy(captureBuffer + captureBufferUsed, row, rowLen);
    captureBufferUsed += rowLen;
    captureBuffer[captureBufferUsed++] = '\n';
    return true;
}

void Icm20948Capture::writerTaskLoop(GwApi *api)
{
    GwLog *logger = api->getLogger();
    char row[600];
    while (true)
    {
        DiagnosticSample sample;
        if (xQueueReceive(queue, &sample, pdMS_TO_TICKS(500)) != pdTRUE)
            continue;

        int n = ImuDiagnostics::formatCsvRow(sample, row, sizeof(row));
        if (n < 0)
        {
            LOG_DEBUG(GwLog::ERROR, "icm20948 capture: row formatting overflowed the buffer, sample dropped");
            continue;
        }

        if (serialEnabled)
            Serial.println(row);

        if (captureActive)
            appendToBuffer(row);

        samplesWritten++;
    }
}
