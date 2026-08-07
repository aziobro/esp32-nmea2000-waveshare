#include "GwIcm20948Task.h"
#include "GwHardware.h"
#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"
#include <N2kMessages.h>
#include <atomic>

#include "GwIcm20948HardwareAdapter.h"
#include "ImuTypes.h"
#include "ImuAngleMath.h"
#include "ImuCoordinateTransform.h"
#include "ImuCalibration.h"
#include "ImuCompass.h"
#include "ImuQuaternion.h"
#include "ImuFusion.h"
#include "ImuHeadingSource.h"
#include "ImuMagMonitor.h"
#include "ImuHeadingFilter.h"
#include "ImuCycleProcessor.h"
#include "ImuGyroCal.h"
#include "ImuMagCal2D.h"
#include "ImuDeviationTable.h"
#include "ImuCalibrationJson.h"
#include "ImuDiagnostics.h"
#include "GwIcm20948CaptureTask.h"
#include "GwIcm20948CalControlTask.h"
#include <math.h>

/*
  Orchestration layer: reads config, drives the hardware adapter, and runs
  every sample through the pure pipeline in lib/icm20948pure to build one
  ImuSolution per cycle, which PGN 127257/127250/127251 are all generated
  from together (never mixing this cycle's roll/pitch with a stale
  heading - see doc/IcmHeadingArchitecture.md for the full processing
  order and the reasoning behind every stage).

  Heading comes from the tilt-compensated software compass or the software
  9-axis fusion candidate, selected/blended by ImuHeadingSource.

  Config migration: every pre-rewrite config key (icmRollInv, icmMagXOff,
  icmHdgOff, etc.) keeps its exact name, default, and meaning - see
  ImuCalibrationOps::migrateFromLegacy(), which builds the calibration
  model directly from them, verified by test to reproduce the old
  hard-iron-only formula exactly. icmSendHdg is unchanged and still the
  master transmit gate, still defaulting off.
*/

#ifndef GWICM20948_SDA_PIN
#define GWICM20948_SDA_PIN -1
#endif
#ifndef GWICM20948_SCL_PIN
#define GWICM20948_SCL_PIN -1
#endif

static HeadingSourceMode parseHeadingMode(const String &s)
{
    if (s == "software_compass")
        return HeadingSourceMode::SoftwareCompass;
    if (s == "software_9axis_fusion")
        return HeadingSourceMode::SoftwareFusion;
    if (s == "auto")
        return HeadingSourceMode::Auto;
    if (s == "diagnostic_only")
        return HeadingSourceMode::DiagnosticOnly;
    return HeadingSourceMode::Auto;
}

static const char *headingSourceName(HeadingSource s)
{
    switch (s)
    {
    case HeadingSource::SoftwareCompass:
        return "software_compass";
    case HeadingSource::SoftwareFusion:
        return "software_9axis_fusion";
    default:
        return "none";
    }
}

static const char *headingQualityName(HeadingQuality q)
{
    switch (q)
    {
    case HeadingQuality::Good:
        return "good";
    case HeadingQuality::Poor:
        return "poor";
    default:
        return "invalid";
    }
}

static const char *headingHoldoverStateName(HeadingHoldoverState s)
{
    switch (s)
    {
    case HeadingHoldoverState::Tracking:
        return "tracking";
    case HeadingHoldoverState::Holdover:
        return "holdover";
    default:
        return "lost";
    }
}

// Thread-safe holder for the values our web request handler serves - the
// handler runs on the webserver's thread while runIcm20948Task updates it,
// same pattern as ExampleWebData in lib/exampletask/GwExampleTask.cpp.
class Icm20948WebData
{
    SemaphoreHandle_t lock;
    bool valid = false;
    double roll = 0, pitch = 0;       // calibrated, degrees
    double rawRoll = 0, rawPitch = 0; // after invert, pre fine-offset, degrees
    double heading = 0;               // final output heading, degrees
    bool headingValid = false;
    double accXg = 0, accYg = 0, accZg = 0; // raw accelerometer, g
    double rotDegPerSec = 0;
    double rotDerivedDegPerSec = 0;
    bool rotDisagrees = false;

    // New diagnostics
    String headingSource = "none";
    String headingQuality = "invalid";
    uint32_t rejectionFlags = 0;
    double magMagnitude = 0;
    double compassHeadingDeg = 0;
    double fusionHeadingDeg = 0;
    bool fusionValid = false;
    MagDisturbanceState magState = MagDisturbanceState::Unknown;
    String magSource = "agmt";
    bool magValid = false;
    double magRawX = 0, magRawY = 0, magRawZ = 0;
    double magAgmtRawX = 0, magAgmtRawY = 0, magAgmtRawZ = 0;
    double magBoatX = 0, magBoatY = 0, magBoatZ = 0;
    double magCorrX = 0, magCorrY = 0, magCorrZ = 0;
    bool headingHoldover = false;
    String headingHoldoverState = "lost";
    uint32_t sensorErrorCount = 0;
    uint32_t sensorReinitCount = 0;
    uint32_t i2cRecoveryCount = 0;
    uint32_t magRecoveryCount = 0;
    uint32_t magNoDataCount = 0;
    uint32_t magOverflowCount = 0;
    uint32_t magZeroCount = 0;
    uint32_t i2cRetryCount = 0;
    String lastIcmStatus = "not started";

public:
    Icm20948WebData() { lock = xSemaphoreCreateMutex(); }
    ~Icm20948WebData() { vSemaphoreDelete(lock); }
    void set(double r, double p, double rawR, double rawP)
    {
        GWSYNCHRONIZED(lock);
        roll = r;
        pitch = p;
        rawRoll = rawR;
        rawPitch = rawP;
        valid = true;
    }
    void setHeading(double h)
    {
        GWSYNCHRONIZED(lock);
        heading = h;
        headingValid = true;
    }
    void setHeadingInvalid()
    {
        GWSYNCHRONIZED(lock);
        headingValid = false;
    }
    void setAccel(double ax, double ay, double az)
    {
        GWSYNCHRONIZED(lock);
        accXg = ax;
        accYg = ay;
        accZg = az;
    }
    void setRot(double degPerSec)
    {
        GWSYNCHRONIZED(lock);
        rotDegPerSec = degPerSec;
    }
    void setRotDiagnostic(double derivedDegPerSec, bool disagrees)
    {
        GWSYNCHRONIZED(lock);
        rotDerivedDegPerSec = derivedDegPerSec;
        rotDisagrees = disagrees;
    }
    void setDiagnostics(HeadingSource src, HeadingQuality q, uint32_t rejFlags, double magMag,
                         double compassHdg, double fusionHdg, bool fusOk,
                         MagDisturbanceState mState, bool holdover, HeadingHoldoverState holdoverState)
    {
        GWSYNCHRONIZED(lock);
        headingSource = headingSourceName(src);
        headingQuality = headingQualityName(q);
        rejectionFlags = rejFlags;
        magMagnitude = magMag;
        compassHeadingDeg = compassHdg;
        fusionHeadingDeg = fusionHdg;
        fusionValid = fusOk;
        magState = mState;
        headingHoldover = holdover;
        headingHoldoverState = headingHoldoverStateName(holdoverState);
    }
    void setMagRaw(const Vec3 &raw, const Vec3 &agmtRaw, const Vec3 &boat, const Vec3 &corrected, const char *source, bool validMag)
    {
        GWSYNCHRONIZED(lock);
        magSource = source ? source : "";
        magValid = validMag;
        magRawX = raw.x;
        magRawY = raw.y;
        magRawZ = raw.z;
        magAgmtRawX = agmtRaw.x;
        magAgmtRawY = agmtRaw.y;
        magAgmtRawZ = agmtRaw.z;
        magBoatX = boat.x;
        magBoatY = boat.y;
        magBoatZ = boat.z;
        magCorrX = corrected.x;
        magCorrY = corrected.y;
        magCorrZ = corrected.z;
    }
    void setCommunicationDiagnostics(uint32_t sensorErrors, uint32_t sensorReinits,
                                     uint32_t i2cRecoveries, uint32_t magRecoveries,
                                     uint32_t magNoData, uint32_t magOverflows,
                                     uint32_t magZeroSamples, uint32_t i2cRetries,
                                     const char *status)
    {
        GWSYNCHRONIZED(lock);
        sensorErrorCount = sensorErrors;
        sensorReinitCount = sensorReinits;
        i2cRecoveryCount = i2cRecoveries;
        magRecoveryCount = magRecoveries;
        magNoDataCount = magNoData;
        magOverflowCount = magOverflows;
        magZeroCount = magZeroSamples;
        i2cRetryCount = i2cRetries;
        lastIcmStatus = status ? status : "";
    }
    void toJson(GwJsonDocument &doc)
    {
        GWSYNCHRONIZED(lock);
        doc["valid"] = valid;
        doc["roll"] = roll;
        doc["pitch"] = pitch;
        doc["rawRoll"] = rawRoll;
        doc["rawPitch"] = rawPitch;
        doc["headingValid"] = headingValid;
        doc["heading"] = heading;
        doc["accX"] = accXg;
        doc["accY"] = accYg;
        doc["accZ"] = accZg;
        doc["rot"] = rotDegPerSec;
        doc["rotDerived"] = rotDerivedDegPerSec;
        doc["rotDisagrees"] = rotDisagrees;
        doc["headingSource"] = headingSource;
        doc["headingQuality"] = headingQuality;
        doc["rejectionFlags"] = rejectionFlags;
        doc["magMagnitude"] = magMagnitude;
        doc["compassHeading"] = compassHeadingDeg;
        doc["fusionHeading"] = fusionHeadingDeg;
        doc["fusionValid"] = fusionValid;
        doc["magDisturbed"] = (magState == MagDisturbanceState::Disturbed);
        doc["headingHoldover"] = headingHoldover;
        doc["headingHoldoverState"] = headingHoldoverState;
        doc["magRawX"] = magRawX;
        doc["magRawY"] = magRawY;
        doc["magRawZ"] = magRawZ;
        doc["magSource"] = magSource;
        doc["magValid"] = magValid;
        doc["magAgmtRawX"] = magAgmtRawX;
        doc["magAgmtRawY"] = magAgmtRawY;
        doc["magAgmtRawZ"] = magAgmtRawZ;
        doc["magBoatX"] = magBoatX;
        doc["magBoatY"] = magBoatY;
        doc["magBoatZ"] = magBoatZ;
        doc["magCorrX"] = magCorrX;
        doc["magCorrY"] = magCorrY;
        doc["magCorrZ"] = magCorrZ;
        doc["sensorErrorCount"] = sensorErrorCount;
        doc["sensorReinitCount"] = sensorReinitCount;
        doc["i2cRecoveryCount"] = i2cRecoveryCount;
        doc["magRecoveryCount"] = magRecoveryCount;
        doc["magNoDataCount"] = magNoDataCount;
        doc["magOverflowCount"] = magOverflowCount;
        doc["magZeroCount"] = magZeroCount;
        doc["i2cRetryCount"] = i2cRetryCount;
        doc["lastIcmStatus"] = lastIcmStatus;
    }
};

// Lightweight, always-on per-cycle timing/resource counters for the
// Performance web panel - see doc/IcmPerformanceReview.md. Deliberately
// not behind a config toggle: the overhead is a handful of micros()/
// FreeRTOS calls per cycle (no allocation), negligible next to the
// hundreds of microseconds the pipeline itself takes, so there's no real
// case for letting it be switched off.
class Icm20948PerfStats
{
    SemaphoreHandle_t lock;
    uint32_t sensorReadUs = 0, sensorReadMaxUs = 0;
    uint32_t processingUs = 0, processingMaxUs = 0;
    uint32_t fusionUs = 0, fusionMaxUs = 0;
    uint32_t loggingEnqueueUs = 0, loggingEnqueueMaxUs = 0;
    uint32_t nmeaSendUs = 0, nmeaSendMaxUs = 0;
    uint32_t totalLoopUs = 0, totalLoopMaxUs = 0;
    uint32_t missedDeadlines = 0;
    uint32_t sensorErrors = 0, sensorReinits = 0;
    uint32_t freeHeapBytes = 0, minFreeHeapEverBytes = 0;
    uint32_t stackHighWaterMarkBytes = 0;

public:
    Icm20948PerfStats() { lock = xSemaphoreCreateMutex(); }
    ~Icm20948PerfStats() { vSemaphoreDelete(lock); }

    void update(uint32_t sensorUs, uint32_t procUs, uint32_t fusUs, uint32_t logUs, uint32_t nmeaUs,
                uint32_t totalUs, bool missedDeadline, uint32_t sensorErrorCount, uint32_t sensorReinitCount,
                uint32_t freeHeap, uint32_t minFreeHeapEver, uint32_t stackHighWaterBytes)
    {
        GWSYNCHRONIZED(lock);
        sensorReadUs = sensorUs;
        if (sensorUs > sensorReadMaxUs) sensorReadMaxUs = sensorUs;
        processingUs = procUs;
        if (procUs > processingMaxUs) processingMaxUs = procUs;
        fusionUs = fusUs;
        if (fusUs > fusionMaxUs) fusionMaxUs = fusUs;
        loggingEnqueueUs = logUs;
        if (logUs > loggingEnqueueMaxUs) loggingEnqueueMaxUs = logUs;
        nmeaSendUs = nmeaUs;
        if (nmeaUs > nmeaSendMaxUs) nmeaSendMaxUs = nmeaUs;
        totalLoopUs = totalUs;
        if (totalUs > totalLoopMaxUs) totalLoopMaxUs = totalUs;
        if (missedDeadline) missedDeadlines++;
        sensorErrors = sensorErrorCount;
        sensorReinits = sensorReinitCount;
        freeHeapBytes = freeHeap;
        minFreeHeapEverBytes = minFreeHeapEver;
        stackHighWaterMarkBytes = stackHighWaterBytes;
    }

    void toJson(GwJsonDocument &doc)
    {
        GWSYNCHRONIZED(lock);
        doc["sensorReadUs"] = sensorReadUs;
        doc["sensorReadMaxUs"] = sensorReadMaxUs;
        doc["processingUs"] = processingUs;
        doc["processingMaxUs"] = processingMaxUs;
        doc["fusionUs"] = fusionUs;
        doc["fusionMaxUs"] = fusionMaxUs;
        doc["loggingEnqueueUs"] = loggingEnqueueUs;
        doc["loggingEnqueueMaxUs"] = loggingEnqueueMaxUs;
        doc["nmeaSendUs"] = nmeaSendUs;
        doc["nmeaSendMaxUs"] = nmeaSendMaxUs;
        doc["totalLoopUs"] = totalLoopUs;
        doc["totalLoopMaxUs"] = totalLoopMaxUs;
        doc["missedDeadlines"] = missedDeadlines;
        doc["sensorErrors"] = sensorErrors;
        doc["sensorReinits"] = sensorReinits;
        doc["freeHeapBytes"] = freeHeapBytes;
        doc["minFreeHeapEverBytes"] = minFreeHeapEverBytes;
        doc["stackHighWaterMarkBytes"] = stackHighWaterMarkBytes;
    }
};

// Split across two call sites with OPPOSITE lifetime requirements, both
// discovered the hard way on real hardware (neither is testable in the
// native env - task scheduling isn't part of it):
//   - initIcm20948() calls capture.startWriterTask(api): addUserTask()
//     only succeeds with the framework's init-phase api instance
//     (isInit=true) - calling it from within an already-running task's
//     own body silently fails (logs an error, returns false, no task
//     ever gets created).
//   - runIcm20948Task() (below) calls capture.begin(api):
//     registerRequestHandler() stores handlers on the CALLING api
//     instance, which GwUserCode deletes the moment the function it was
//     constructed for returns - fine from a task that runs forever
//     (never returns), fatal from the one-shot initIcm20948 (handlers
//     vanish immediately, requests 404).
// Either half called from the wrong site is a silent failure, not a
// compile error - hence this file-scope object (rather than a
// runIcm20948Task-local one, like webData/perfStats) so both call sites
// can reach the same instance.
static Icm20948Capture capture;

// Self-contained software watchdog for runIcm20948Task(). Real, bench-
// confirmed bug (not hypothetical): a Qwiic cable flexed during handheld
// tumbling can leave the I2C bus in a bad mid-transaction state that the
// ESP32 Arduino Wire driver never times out of, despite
// GwIcm20948HardwareAdapter::begin()'s own Wire.setTimeOut(1000) - watched
// this hang the main task for 20-40+ seconds with ZERO log output (not a
// crash, not a logged I2C error - a genuine silent infinite block deep in
// the driver stack). Since the stuck task can't rescue itself, a second
// tiny task watches a heartbeat timestamp and force-restarts the device if
// it goes stale - converts a hang requiring physical/manual power-cycling
// into a ~10s self-recovering reboot. Must be started via addUserTask()
// from initIcm20948() (init-phase api), not from within runIcm20948Task -
// same constraint as capture.startWriterTask(), see its doc comment above.
// std::atomic (not plain volatile) - real bug found live 2026-08-06: core-
// pinning the watchdog to a dedicated core (confirmed working via direct
// core-ID logging) still didn't let it recover a real hang. `volatile`
// only blocks compiler-level reordering/caching of a single access - it
// is not a cross-core memory barrier, so a write on the sensor task's
// core is not guaranteed to become visible to the watchdog task running
// on the other core in any bounded time. std::atomic's default
// sequentially-consistent ops emit the actual memory-barrier instructions
// needed for that guarantee on this dual-core Xtensa target.
static std::atomic<unsigned long> g_icm20948LastHeartbeatMs{0};
static std::atomic<bool> g_icm20948WatchdogArmed{false};
static const unsigned long ICM20948_WATCHDOG_TIMEOUT_MS = 8000;
static const unsigned long ICM20948_SENSOR_STALE_RECOVER_MS = 1500;
static const unsigned long ICM20948_SENSOR_REINIT_BACKOFF_MS = 1500;
static const unsigned long ICM20948_SENSOR_REINIT_WINDOW_MS = 10000;
static const unsigned long ICM20948_SENSOR_REINIT_SUPPRESS_MS = 60000;
static const unsigned long ICM20948_MAG_BRIDGE_RECOVER_BACKOFF_MS = 1000;
static const uint8_t ICM20948_SENSOR_REINIT_WINDOW_LIMIT = 3;
static const uint8_t ICM20948_MAG_BRIDGE_RECOVER_ERRORS = 5;

// A cheap "where in the loop was I last" marker, updated at each key step
// below with a plain string-literal pointer assignment (no formatting, no
// logging - negligible cost every cycle). Read by the watchdog only in
// the rare case it's about to force a restart, so a restart's log message
// says exactly which call the task was in rather than just that it froze
// for the real hang this helped root-cause live during bench verification.
static volatile const char *g_icm20948LastCheckpoint = "not started";

static void icm20948WatchdogTaskEntry(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "icm20948 watchdog task running on core %d", xPortGetCoreID());
    while (true)
    {
        delay(2000);
        if (!g_icm20948WatchdogArmed)
            continue;
        unsigned long age = millis() - g_icm20948LastHeartbeatMs;
        if (age > ICM20948_WATCHDOG_TIMEOUT_MS)
        {
            // Real bug found live, 2026-08-06: GwLog::logDebug()/flush()
            // both do xSemaphoreTake(locker, portMAX_DELAY) on a single
            // GLOBAL mutex shared by every LOG_DEBUG call in the whole
            // firmware. The whole point of this watchdog is to recover
            // from the sensor task hanging mid-operation - if that hang
            // happened while the sensor task held this same lock (e.g.
            // mid-LOG_DEBUG when the I2C call it was about to make wedged),
            // then THIS task's own LOG_DEBUG call a few lines below would
            // block forever waiting for a lock that will never be freed,
            // and the restart below would never happen either - exactly
            // what was observed on real hardware: the watchdog reliably
            // detected the stale heartbeat but never actually restarted,
            // even after confirming (via core-ID logging) it was correctly
            // isolated on its own CPU core. Write directly to the serial
            // port instead, bypassing GwLog's mutex entirely, so a wedged
            // log lock can never block the restart itself.
            USBSerial.printf("icm20948 task watchdog: no heartbeat for %lu ms - forcing restart. Last checkpoint: %s\n",
                              age, g_icm20948LastCheckpoint);
            USBSerial.flush();
            delay(100);
            ESP.restart();
        }
    }
}

static void runIcm20948Task(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "icm20948 task starting, sda=%d scl=%d", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
    LOG_DEBUG(GwLog::LOG, "icm20948 sensor task running on core %d", xPortGetCoreID());

    Icm20948WebData webData;
    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                {
        GwJsonDocument doc(1152);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    Icm20948PerfStats perfStats;
    api->registerRequestHandler("perfStatus", [&perfStats](AsyncWebServerRequest *request)
                                 {
        GwJsonDocument doc(512);
        perfStats.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    // Registers capControl/capStatus/capDownload - must run from here
    // (see capture's declaration comment above), NOT from initIcm20948
    // (where the writer task itself was already started).
    capture.begin(api);

    GwIcm20948HardwareAdapter hw;
    while (!hw.begin(logger, GWICM20948_SDA_PIN, GWICM20948_SCL_PIN))
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found - retrying in 5s");
        delay(5000);
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found, starting attitude updates");
    GwConfigHandler *config = api->getConfig();

    Icm20948CalControl calControl;
    calControl.begin(api, config);

    int accRangeIdx = 0, gyrRangeIdx = 0;
    config->getValue(accRangeIdx, GwConfigDefinitions::icmAccRange, 0);
    config->getValue(gyrRangeIdx, GwConfigDefinitions::icmGyrRange, 0);
    hw.setFullScale(logger, accRangeIdx, gyrRangeIdx);

    int rateHz = 10;
    config->getValue(rateHz, GwConfigDefinitions::icmRateHz, 10);
    if (rateHz < 1)
        rateHz = 1;
    unsigned long loopDelayMs = 1000UL / (unsigned long)rateHz;
    LOG_DEBUG(GwLog::LOG, "ICM20948 update rate %d Hz (delay %lu ms)", rateHz, loopDelayMs);

    unsigned char sid = 0;
    unsigned long taskStartMs = millis();
    unsigned long lastCycleMs = millis();

    uint8_t consecutiveMagInvalid = 0;
    uint32_t sensorReinitCount = 0;
    unsigned long noDataSinceMs = 0;
    unsigned long agmtFailSinceMs = 0;
    unsigned long lastSensorRecoveryMs = 0;
    unsigned long sensorRecoveryWindowStartMs = 0;
    unsigned long sensorRecoverySuppressedUntilMs = 0;
    unsigned long lastMagBridgeRecoveryMs = 0;
    uint8_t sensorRecoveryAttemptsInWindow = 0;

    // The actual heading/attitude pipeline (fusion filter, source
    // selector, mag disturbance monitor, heading filter, ROT
    // cross-check state) lives in ImuCycleProcessor now - the SAME class
    // GwIcm20948ReplayTask uses, so replay runs provably the same code as
    // this real hardware loop rather than a parallel reimplementation.
    ImuCycleProcessor cycleProcessor;
    uint32_t diagSampleSeq = 0;

    // Carried forward between loop ticks so the Attitude PGN's Yaw field
    // is never left N/A when a heading is available - some chartplotters
    // treat the whole message as stale otherwise (see the Garmin bridge
    // investigation this fix came from).
    double lastHeadingRad = 0;
    bool haveLastHeading = false;

    auto resetRuntimeSamples = [&]() {
        consecutiveMagInvalid = 0;
        lastCycleMs = millis();
    };

    auto updateCommunicationDiagnostics = [&]() {
        webData.setCommunicationDiagnostics(
            hw.sensorErrorCount(), sensorReinitCount,
            hw.i2cRecoveryCount(), hw.magRecoveryCount(), hw.magNoDataCount(),
            hw.magOverflowCount(), hw.magZeroCount(), hw.i2cRetryCount(),
            hw.lastStatusString());
    };

    auto recoverSensor = [&](const char *reason) -> bool {
        unsigned long recoveryStartMs = millis();
        if (sensorRecoverySuppressedUntilMs != 0 && recoveryStartMs < sensorRecoverySuppressedUntilMs)
            return false;
        if (recoveryStartMs - lastSensorRecoveryMs < ICM20948_SENSOR_REINIT_BACKOFF_MS)
            return false;

        if (sensorRecoveryWindowStartMs == 0 ||
            recoveryStartMs - sensorRecoveryWindowStartMs > ICM20948_SENSOR_REINIT_WINDOW_MS)
        {
            sensorRecoveryWindowStartMs = recoveryStartMs;
            sensorRecoveryAttemptsInWindow = 0;
        }
        sensorRecoveryAttemptsInWindow++;
        if (sensorRecoveryAttemptsInWindow > ICM20948_SENSOR_REINIT_WINDOW_LIMIT)
        {
            sensorRecoverySuppressedUntilMs = recoveryStartMs + ICM20948_SENSOR_REINIT_SUPPRESS_MS;
            sensorRecoveryAttemptsInWindow = 0;
            sensorRecoveryWindowStartMs = recoveryStartMs;
            LOG_DEBUG(GwLog::ERROR, "ICM20948 sensor reinitialize suppressed for %lu ms after repeated attempts at %s",
                      ICM20948_SENSOR_REINIT_SUPPRESS_MS, reason);
            return false;
        }

        lastSensorRecoveryMs = recoveryStartMs;
        g_icm20948LastCheckpoint = reason;
        g_icm20948LastHeartbeatMs = recoveryStartMs;
        sensorReinitCount++;

        bool ok = hw.reinitialize(logger, GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
        if (ok)
        {
            hw.setFullScale(logger, accRangeIdx, gyrRangeIdx);
            resetRuntimeSamples();
            noDataSinceMs = 0;
            agmtFailSinceMs = 0;
        }
        g_icm20948LastHeartbeatMs = millis();
        return ok;
    };

    // Arm the watchdog only now - startup above (I2C scan, up to 6 sensor
    // init retries at 300ms each) can legitimately take a few seconds and
    // shouldn't be mistaken for a hang.
    g_icm20948LastHeartbeatMs = millis();
    g_icm20948WatchdogArmed = true;

    while (true)
    {
        g_icm20948LastCheckpoint = "before dataReady";
        delay(loopDelayMs);
        if (!hw.dataReady())
        {
            unsigned long staleNowMs = millis();
            if (noDataSinceMs == 0)
                noDataSinceMs = staleNowMs;
            if (staleNowMs - noDataSinceMs > ICM20948_SENSOR_STALE_RECOVER_MS)
                recoverSensor("recovering after stale dataReady");
            updateCommunicationDiagnostics();
            // dataReady() returned, so the task is alive even if the sensor is
            // currently stale. Keep the watchdog reserved for calls that block.
            g_icm20948LastHeartbeatMs = millis();
            continue;
        }
        noDataSinceMs = 0;
        g_icm20948LastCheckpoint = "after dataReady, before readAGMT";

        unsigned long loopStartUs = micros();
        unsigned long sensorReadUs = 0;
        unsigned long nmeaSendUs = 0;

        unsigned long sensorReadStartUs = micros();
        if (!hw.readAGMT())
        {
            g_icm20948LastCheckpoint = "readAGMT failed";
            unsigned long failNowMs = millis();
            if (agmtFailSinceMs == 0)
                agmtFailSinceMs = failNowMs;
            if (failNowMs - agmtFailSinceMs > ICM20948_SENSOR_STALE_RECOVER_MS)
                recoverSensor("recovering after readAGMT failures");
            updateCommunicationDiagnostics();
            // readAGMT() returned a failure status rather than hanging. That is
            // a sensor fault, not a task stall, so do not let the watchdog turn
            // the recovery backoff into a whole-device reset loop.
            g_icm20948LastHeartbeatMs = millis();
            continue;
        }
        agmtFailSinceMs = 0;
        sensorRecoveryAttemptsInWindow = 0;
        sensorRecoveryWindowStartMs = millis();
        sensorRecoverySuppressedUntilMs = 0;
        g_icm20948LastCheckpoint = "after readAGMT";

        unsigned long nowMs = millis();
        double dtSec = (nowMs - lastCycleMs) / 1000.0;
        if (dtSec <= 0 || dtSec > 1.0) // clamp - a stalled loop shouldn't inject a huge fusion step
            dtSec = 1.0 / (double)rateHz;
        lastCycleMs = nowMs;

        // --- Re-read config every cycle (existing pattern - changes on
        // the Config page take effect immediately, no reboot needed for
        // anything except the one-time hardware settings above). ---
        int orientationIdx = 0;
        config->getValue(orientationIdx, GwConfigDefinitions::icmOrientation, 0);
        MountOrientation orientation = static_cast<MountOrientation>(orientationIdx);

        float rollOffsetDeg = 0, pitchOffsetDeg = 0;
        bool rollInvert = false, pitchInvert = false, sendAttitude = true;
        config->getValue(rollOffsetDeg, GwConfigDefinitions::icmRollOff, 0.0f);
        config->getValue(pitchOffsetDeg, GwConfigDefinitions::icmPitchOff, 0.0f);
        config->getValue(rollInvert, GwConfigDefinitions::icmRollInv, false);
        config->getValue(pitchInvert, GwConfigDefinitions::icmPitchInv, false);
        config->getValue(sendAttitude, GwConfigDefinitions::icmSendAtt, true);

        float hdgOffsetDeg = 0;
        bool hdgInvert = false, sendHeading = false;
        config->getValue(hdgOffsetDeg, GwConfigDefinitions::icmHdgOff, 0.0f);
        config->getValue(hdgInvert, GwConfigDefinitions::icmHdgInv, false);
        config->getValue(sendHeading, GwConfigDefinitions::icmSendHdg, false);

        float magXOffset = 0, magYOffset = 0;
        config->getValue(magXOffset, GwConfigDefinitions::icmMagXOff, 0.0f);
        config->getValue(magYOffset, GwConfigDefinitions::icmMagYOff, 0.0f);

        String headingModeStr = "auto";
        config->getValue(headingModeStr, GwConfigDefinitions::icmHeadingMode, "auto");
        HeadingSourceMode headingMode = parseHeadingMode(headingModeStr);

        int transitionMs = 1000;
        config->getValue(transitionMs, GwConfigDefinitions::icmSrcTransMs, 1000);

        bool filterEnabled = false;
        float filterTimeConstant = 1.0f;
        config->getValue(filterEnabled, GwConfigDefinitions::icmHdgFiltEn, false);
        config->getValue(filterTimeConstant, GwConfigDefinitions::icmHdgFiltTau, 1.0f);

        int hdgHoldoverMs = 5000;
        config->getValue(hdgHoldoverMs, GwConfigDefinitions::icmHdgHoldMs, 5000);

        bool deviationEnabled = false;
        config->getValue(deviationEnabled, GwConfigDefinitions::icmDevEnable, false);

        bool sendRot = true, rotInvert = false;
        config->getValue(sendRot, GwConfigDefinitions::icmSendRot, true);
        config->getValue(rotInvert, GwConfigDefinitions::icmRotInv, false);
        float rotFiltAlpha = 1.0f;
        config->getValue(rotFiltAlpha, GwConfigDefinitions::icmRotFiltAlpha, 1.0f);

        // --- Raw samples, transformed into boat frame. At the default
        // MountOrientation::Forward this transform is the identity, so
        // every value below is bit-for-bit what the pre-rewrite code
        // computed directly from the raw accX/accY/accZ etc. ---
        Vec3 accelBoat = ImuCoordinateTransform::toBoatFrame(hw.readAccelG(), orientation);
        Vec3 gyroBoat = ImuCoordinateTransform::toBoatFrame(hw.readGyroDegPerSec(), orientation);
        Vec3 agmtMagRaw = hw.readMagRaw();
        Vec3 magRaw = agmtMagRaw;
        const char *magSource = "agmt";
        bool magValid = hw.magValid();
        Vec3 magBoat = ImuCoordinateTransform::toBoatFrame(magRaw, orientation);
        sensorReadUs += micros() - sensorReadStartUs;
        webData.setAccel(accelBoat.x, accelBoat.y, accelBoat.z);

        // Successful read cycle heartbeat. Failure paths above also refresh
        // the heartbeat after the I2C call returns, so the watchdog only
        // escalates genuine blocking stalls.
        g_icm20948LastHeartbeatMs = millis();

        if (!magValid)
        {
            if (consecutiveMagInvalid < 255)
                consecutiveMagInvalid++;
            if (consecutiveMagInvalid >= ICM20948_MAG_BRIDGE_RECOVER_ERRORS)
            {
                g_icm20948LastCheckpoint = "before magnetometer bridge recovery";
                if (nowMs - lastMagBridgeRecoveryMs >= ICM20948_MAG_BRIDGE_RECOVER_BACKOFF_MS &&
                    hw.recoverMagBridge())
                {
                    consecutiveMagInvalid = 0;
                    lastMagBridgeRecoveryMs = millis();
                    LOG_DEBUG(GwLog::LOG, "ICM20948 magnetometer bridge recovered after %u invalid samples",
                              ICM20948_MAG_BRIDGE_RECOVER_ERRORS);
                }
                g_icm20948LastHeartbeatMs = millis();
            }
        }
        else
        {
            consecutiveMagInvalid = 0;
        }

        // --- Calibration (moved ahead of Rate of Turn so gyro bias
        // correction applies before the gyro vector is used anywhere -
        // ImuCalibrationOps::applyGyro was previously never called, see
        // doc/IcmImplementationAudit.md). Prefers the advanced calibration
        // saved via the web UI's Calibration/Gyroscope panels (persisted
        // as icmCalJson - see GwIcm20948CalControlTask.h) over the legacy
        // hard-iron-only fields, falling back to migrateFromLegacy() when
        // no advanced calibration has been saved yet. The expensive JSON
        // parse only re-runs when the stored string actually changes, not
        // every cycle - see the cached statics below. ---
        static String lastCalJsonStr = "\x01"; // sentinel - never a real config value, forces the first-cycle parse
        static ImuCalibration cachedCal;
        static DeviationTable cachedDeviationTable;
        static bool cachedDeviationTableEnabled = false;
        {
            String calJsonStr;
            config->getValue(calJsonStr, GwConfigDefinitions::icmCalJson, "");
            if (calJsonStr != lastCalJsonStr)
            {
                lastCalJsonStr = calJsonStr;
                bool parsed = false;
                if (calJsonStr.length() > 0)
                {
                    MountOrientation ignoredOrientation; // icmOrientation config remains authoritative
                    std::string err;
                    parsed = ImuCalibrationJson::importJson(std::string(calJsonStr.c_str()), cachedCal, ignoredOrientation,
                                                             cachedDeviationTable, cachedDeviationTableEnabled, err);
                }
                if (!parsed)
                {
                    cachedCal = ImuCalibrationOps::migrateFromLegacy(magXOffset, magYOffset, hdgOffsetDeg);
                    cachedDeviationTable = DeviationTable();
                }
            }
        }
        // Use cachedCal directly rather than copying it into a local first
        // (applyGyro takes it by const reference, and cycleIn.cal below
        // can be assigned straight from it too) - one ImuCalibration copy
        // per cycle instead of two (see doc/IcmPerformanceReview.md).
        Vec3 gyroCal = ImuCalibrationOps::applyGyro(gyroBoat, cachedCal);

        // --- Feed the interactive calibration engines (Calibration/
        // Gyroscope web panels) - cheap, no-ops unless a swing/stationary
        // capture is actively running. Uses the RAW (pre-calibration)
        // gyro/mag boat-frame vectors, since these engines measure
        // absolute bias/scale, not a residual on top of whatever's
        // already applied - see GwIcm20948CalControlTask.h. ---
        calControl.feedGyroSample(gyroBoat, accelBoat.norm());
        calControl.feedMagSample(magBoat.x, magBoat.y);

        // --- Run the shared per-cycle pipeline (ImuCycleProcessor -
        // see lib/icm20948pure/ImuCycleProcessor.h). This is the SAME
        // code debug replay uses (GwIcm20948ReplayTask), not a parallel
        // reimplementation - everything from gyro/mag calibration through
        // heading-source selection and final heading correction lives
        // there now. ---
        ImuCycleInput cycleIn;
        cycleIn.accelBoat = accelBoat;
        cycleIn.gyroBoat = gyroBoat;
        cycleIn.magBoat = magBoat;
        cycleIn.magValid = magValid;
        cycleIn.dtSec = dtSec;
        cycleIn.nowMs = nowMs;
        cycleIn.taskStartMs = taskStartMs;
        cycleIn.cal = cachedCal;
        cycleIn.deviationTable = cachedDeviationTable;
        cycleIn.deviationEnabled = deviationEnabled;
        cycleIn.headingMode = headingMode;
        cycleIn.transitionMs = (uint32_t)transitionMs;
        cycleIn.rollInvert = rollInvert;
        cycleIn.pitchInvert = pitchInvert;
        cycleIn.rollOffsetDeg = rollOffsetDeg;
        cycleIn.pitchOffsetDeg = pitchOffsetDeg;
        cycleIn.hdgInvert = hdgInvert;
        cycleIn.hdgOffsetDeg = hdgOffsetDeg;
        cycleIn.filterEnabled = filterEnabled;
        cycleIn.filterTimeConstantSec = filterTimeConstant;
        cycleIn.rotInvert = rotInvert;
        cycleIn.rotFiltAlpha = rotFiltAlpha;
        cycleIn.headingHoldoverConfig.maxHoldoverMs = (double)hdgHoldoverMs;

        unsigned long processingStartUs = micros();
        ImuCycleOutput cycleOut = cycleProcessor.process(cycleIn);
        unsigned long processingUs = micros() - processingStartUs;

        webData.setRot(cycleOut.rotDegPerSec);
        if (sendRot)
        {
            unsigned long t0 = micros();
            tN2kMsg rotMsg;
            SetN2kRateOfTurn(rotMsg, sid, radians(cycleOut.rotDegPerSec));
            api->sendN2kMessage(rotMsg);
            sid = (sid + 1) % 252;
            nmeaSendUs += micros() - t0;
        }

        webData.setMagRaw(magRaw, agmtMagRaw, magBoat, cycleOut.magCorrected, magSource, magValid);

        if (cycleOut.attitudeValid)
        {
            webData.set(cycleOut.rollDeg, cycleOut.pitchDeg, cycleOut.rawRollDeg, cycleOut.rawPitchDeg);

            api->setCalibrationValue(GwConfigDefinitions::icmRollOff, cycleOut.rawRollDeg);
            api->setCalibrationValue(GwConfigDefinitions::icmPitchOff, cycleOut.rawPitchDeg);

            if (sendAttitude)
            {
                unsigned long t0 = micros();
                tN2kMsg msg;
                SetN2kAttitude(msg, sid, haveLastHeading ? lastHeadingRad : N2kDoubleNA, radians(cycleOut.pitchDeg), radians(cycleOut.rollDeg));
                api->sendN2kMessage(msg);
                sid = (sid + 1) % 252;
                nmeaSendUs += micros() - t0;
            }
        }

        // --- Compass calibration preview (legacy hard-iron tracking,
        // unchanged behavior/formula from the pre-rewrite code). ---
        {
            static double magXMin = 1e6, magXMax = -1e6, magYMin = 1e6, magYMax = -1e6;
            if (magBoat.x < magXMin) magXMin = magBoat.x;
            if (magBoat.x > magXMax) magXMax = magBoat.x;
            if (magBoat.y < magYMin) magYMin = magBoat.y;
            if (magBoat.y > magYMax) magYMax = magBoat.y;
            api->setCalibrationValue(GwConfigDefinitions::icmMagXOff, (magXMin + magXMax) / 2.0);
            api->setCalibrationValue(GwConfigDefinitions::icmMagYOff, (magYMin + magYMax) / 2.0);
        }

        if (cycleOut.headingValid)
        {
            webData.setHeading(cycleOut.headingDeg);
            lastHeadingRad = radians(cycleOut.headingDeg);
            haveLastHeading = true;
            api->setCalibrationValue(GwConfigDefinitions::icmHdgOff, cycleOut.preCorrectionHeadingDeg);

            // headingHoldover excluded on purpose: a gyro-dead-reckoned
            // estimate is not a sensor reading and is never sent as PGN
            // 127250, regardless of "Send Magnetic Heading" - see
            // doc/IcmHeadingValidityAudit.md.
            if (sendHeading && headingMode != HeadingSourceMode::DiagnosticOnly && !cycleOut.headingHoldover)
            {
                unsigned long t0 = micros();
                tN2kMsg hdgMsg;
                SetN2kMagneticHeading(hdgMsg, sid, radians(cycleOut.headingDeg));
                api->sendN2kMessage(hdgMsg);
                sid = (sid + 1) % 252;
                nmeaSendUs += micros() - t0;
            }

            webData.setRotDiagnostic(cycleOut.rotDerivedDegPerSec, cycleOut.rotDisagrees);
        }
        else
        {
            webData.setHeadingInvalid();
        }

        webData.setDiagnostics(cycleOut.headingSource, cycleOut.headingQuality, cycleOut.rejectionFlags, cycleOut.magMagnitude,
                                cycleOut.rawCompassHeadingDeg, cycleOut.rawFusionHeadingDeg,
                                cycleOut.fusionCandidateValid, cycleOut.magDisturbanceState, cycleOut.headingHoldover, cycleOut.headingHoldoverState);
        updateCommunicationDiagnostics();

        // --- Diagnostic CSV capture (Phase 2) - offerSample() is a
        // cheap, non-blocking queue push that no-ops entirely unless
        // capture or serial-CSV output is actually active, so this costs
        // nothing in the common case. ---
        unsigned long loggingStartUs = micros();
        {
            DiagnosticSample sample;
            sample.timestampMs = nowMs;
            sample.sampleSequence = diagSampleSeq++;
            sample.accelRaw = hw.readAccelG();
            sample.gyroRaw = hw.readGyroDegPerSec();
            sample.magRaw = magRaw;
            sample.accelBoat = accelBoat;
            sample.gyroBoat = gyroBoat;
            sample.magBoat = magBoat;
            sample.magCorrected = cycleOut.magCorrected;
            sample.magMagnitude = cycleOut.magMagnitude;
            sample.compassHeadingDeg = cycleOut.rawCompassHeadingDeg;
            sample.fusionHeadingDeg = cycleOut.rawFusionHeadingDeg;
            sample.outputHeadingDeg = cycleOut.headingValid ? cycleOut.headingDeg : -1.0;
            sample.rateOfTurnDegPerSec = cycleOut.rotDegPerSec;
            sample.activeSource = cycleOut.headingSource;
            sample.headingQuality = cycleOut.headingQuality;
            sample.rejectionFlags = cycleOut.rejectionFlags;
            sample.sensorErrorCount = hw.sensorErrorCount();
            capture.offerSample(sample);
        }
        unsigned long loggingEnqueueUs = micros() - loggingStartUs;

        // --- Performance instrumentation (Phase 7) - see
        // doc/IcmPerformanceReview.md. Cheap: a handful of already-taken
        // micros() deltas plus two FreeRTOS heap/stack queries, no
        // allocation. ---
        unsigned long totalLoopUs = micros() - loopStartUs;
        bool missedDeadline = totalLoopUs > (loopDelayMs * 1000UL);
        perfStats.update(sensorReadUs, processingUs, (uint32_t)cycleOut.fusionDurationUs, loggingEnqueueUs, nmeaSendUs,
                          totalLoopUs, missedDeadline, hw.sensorErrorCount(), sensorReinitCount,
                          (uint32_t)xPortGetFreeHeapSize(), (uint32_t)xPortGetMinimumEverFreeHeapSize(),
                          (uint32_t)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
    }
}

void initIcm20948(GwApi *api)
{
    GwLog *logger = api->getLogger();
    if (GWICM20948_SDA_PIN < 0 || GWICM20948_SCL_PIN < 0)
    {
        LOG_DEBUG(GwLog::LOG, "no ICM20948 pins defined for this board, task not started");
        return;
    }
    api->addCapability("icm20948", "true");
    // Must run here (init phase, isInit=true) not from within
    // runIcm20948Task - see Icm20948Capture::startWriterTask's doc
    // comment. The complementary capture.begin(api) call (HTTP handler
    // registration) is in runIcm20948Task instead, for the opposite
    // reason - see its doc comment too.
    capture.startWriterTask(api);
    // Must also run here (init phase) - see icm20948WatchdogTaskEntry's
    // doc comment above runIcm20948Task().
    //
    // Both this task and runIcm20948Task below are pinned to opposite
    // CPU cores (real bug found live, 2026-08-06: three separate hangs
    // during physical handling, watchdog never fired despite being
    // confirmed created and running during bench verification).
    // Root cause: both tasks were created via the same unpinned
    // xTaskCreate call, so FreeRTOS's SMP scheduler was free to place
    // them on the same core - if the I2C hang is a genuine hardware-
    // register busy-spin deep in the driver stack that never yields
    // (matching this watchdog's own original doc comment), it can
    // starve everything else scheduled on that core regardless of
    // nominal task priority. Pinning to different cores guarantees the
    // watchdog keeps running even if the sensor task's own core is
    // fully wedged. Core 1 for the sensor task matches arduino-esp32's
    // own conventional default placement for the main loop task; core 0
    // (shared with the WiFi/BT controller's own tasks) is fine for the
    // watchdog since it only wakes every 2s to check a timestamp -
    // negligible contention.
    api->addUserTask(icm20948WatchdogTaskEntry, String("icm20948Wdt"), 2000, /*coreId=*/0);
    // Stack is 16000 bytes - unchanged from the pre-rewrite code (the
    // intermittent cold-boot crash this project hit was an I2C/WiFi boot
    // race, not stack exhaustion - see GwIcm20948HardwareAdapter::begin()).
    api->addUserTask(runIcm20948Task, String("icm20948Task"), 16000, /*coreId=*/1);
}
