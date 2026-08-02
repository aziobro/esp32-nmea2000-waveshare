#pragma once
#include "GwApi.h"
#include "ImuDiagnostics.h"

/*
  Runtime CSV diagnostic capture. The IMU task only ever calls
  offerSample() - a bounded, non-blocking FreeRTOS queue push (drops the
  sample rather than blocking if the queue is full) - so nothing in the
  high-rate sensor loop can stall on serial/formatting/buffer work. All
  of that happens on a separate writer task (its own FreeRTOS task,
  spawned by begin()) that drains the queue at its own pace.

  Two outputs, both optional and independently controlled:
  - Serial CSV: each row printed to the USB serial console as it's
    drained from the queue.
  - RAM capture buffer: a single pre-allocated (at "start", not per
    sample) buffer that accumulated rows are appended to, downloadable
    over HTTP as text/csv. This project has no existing writable
    filesystem/SD support (checked - none found), so this is the
    "or otherwise retrieve" alternative the capture spec explicitly
    allows for rather than inventing a new flash filesystem layer.
    Bounded by whichever of max-duration / max-size is configured,
    whichever is hit first stops capture automatically.
*/
class Icm20948Capture
{
public:
    // Creates the queue, spawns the writer task, and registers the HTTP
    // control/download endpoints under the given api. Call once from
    // initIcm20948.
    void begin(GwApi *api);

    // Non-blocking; called every IMU loop cycle. Internally rate-limits
    // to the configured capture rate (which can be lower than the
    // sensor's own processing rate) and no-ops entirely if capture
    // isn't active and serial-CSV isn't enabled.
    void offerSample(const DiagnosticSample &sample);

    // Entry point for the writer task spawned by begin() - public only
    // because GwUserTaskFunction (see GwApi.h) is a plain function
    // pointer that can't capture `this`, so a free function reaches this
    // instance via a file-scope pointer set in begin() and calls this
    // directly. Not meant to be called from anywhere else.
    void writerTaskLoop(GwApi *api);

private:
    QueueHandle_t queue = nullptr;
    static const int QUEUE_CAPACITY = 64;

    // Stats - written only from the writer task except samplesOffered/
    // samplesDropped (written from the IMU task's offerSample(), read
    // from the writer task/HTTP handler) and queueHighWaterMark (same) -
    // all plain counters, not floats, so torn reads are self-correcting
    // (never a saved/loaded invariant, just monotonically-increasing
    // display counters) and not worth a lock for.
    volatile uint32_t samplesOffered = 0;
    volatile uint32_t samplesDropped = 0;
    volatile uint32_t queueHighWaterMark = 0;
    volatile uint32_t samplesWritten = 0;

    volatile bool serialEnabled = false;
    volatile bool captureActive = false;
    volatile int rateHz = 10;
    volatile int maxDurationSec = 60;
    volatile int maxKB = 128;

    unsigned long lastOfferMs = 0;
    unsigned long captureStartMs = 0;

    char *captureBuffer = nullptr;
    size_t captureBufferCapacity = 0;
    size_t captureBufferUsed = 0;

    void startCapture();
    void stopCapture();
    void clearCapture();
    bool appendToBuffer(const char *row);
};
