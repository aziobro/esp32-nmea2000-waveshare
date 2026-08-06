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

/*
  Orchestration layer: reads config, drives the hardware adapter, and runs
  every sample through the pure pipeline in lib/icm20948pure to build one
  ImuSolution per cycle, which PGN 127257/127250/127251 are all generated
  from together (never mixing this cycle's roll/pitch with a stale
  heading - see doc/IcmHeadingArchitecture.md for the full processing
  order and the reasoning behind every stage).

  DMP yaw is now one candidate heading source among three (DMP, software
  compass, software fusion), validated and selected/blended by
  ImuHeadingSource rather than trusted unconditionally the moment
  icmUseDmp is on - this is the actual fix for this task's original
  design issue (a fixed offset can't correct hard-iron/soft-iron/
  axis-mapping/tilt/heading-dependent error, and DMP bypassed the
  compass calibration entirely).

  Config migration: every pre-rewrite config key (icmRollInv, icmMagXOff,
  icmHdgOff, etc.) keeps its exact name, default, and meaning - see
  ImuCalibrationOps::migrateFromLegacy(), which builds the calibration
  model directly from them, verified by test to reproduce the old
  hard-iron-only formula exactly. All NEW config (heading-source mode,
  orientation, filter/threshold settings) defaults to values that
  reproduce today's behavior: icmHeadingMode defaults to "dmp" (not
  "auto") specifically so introducing software fusion as a new candidate
  doesn't change a currently-deployed, already-tuned unit's output the
  moment this firmware lands - "auto" (which prefers fusion) is an
  explicit opt-in. icmSendHdg is unchanged and still the master transmit
  gate, still defaulting off.
*/

#ifndef GWICM20948_SDA_PIN
#define GWICM20948_SDA_PIN -1
#endif
#ifndef GWICM20948_SCL_PIN
#define GWICM20948_SCL_PIN -1
#endif

static HeadingSourceMode parseHeadingMode(const String &s)
{
    if (s == "dmp")
        return HeadingSourceMode::Dmp;
    if (s == "software_compass")
        return HeadingSourceMode::SoftwareCompass;
    if (s == "software_9axis_fusion")
        return HeadingSourceMode::SoftwareFusion;
    if (s == "auto")
        return HeadingSourceMode::Auto;
    if (s == "diagnostic_only")
        return HeadingSourceMode::DiagnosticOnly;
    return HeadingSourceMode::Dmp; // unrecognized value - fail toward the conservative default
}

static const char *headingSourceName(HeadingSource s)
{
    switch (s)
    {
    case HeadingSource::Dmp:
        return "dmp";
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
    bool dmpActive = false;
    double rotDegPerSec = 0;
    double rotDerivedDegPerSec = 0;
    bool rotDisagrees = false;

    // New diagnostics
    String headingSource = "none";
    String headingQuality = "invalid";
    uint32_t rejectionFlags = 0;
    double magMagnitude = 0;
    double dmpHeadingDeg = 0;
    bool dmpHeadingValid = false;
    double compassHeadingDeg = 0;
    double fusionHeadingDeg = 0;
    bool fusionValid = false;
    MagDisturbanceState magState = MagDisturbanceState::Unknown;
    double magRawX = 0, magRawY = 0, magRawZ = 0;
    double magBoatX = 0, magBoatY = 0, magBoatZ = 0;
    double magCorrX = 0, magCorrY = 0, magCorrZ = 0;
    bool headingHoldover = false;
    String headingHoldoverState = "lost";

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
    void setDmpActive(bool active)
    {
        GWSYNCHRONIZED(lock);
        dmpActive = active;
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
                         double dmpHdg, bool dmpValid, double compassHdg, double fusionHdg, bool fusOk,
                         MagDisturbanceState mState, bool holdover, HeadingHoldoverState holdoverState)
    {
        GWSYNCHRONIZED(lock);
        headingSource = headingSourceName(src);
        headingQuality = headingQualityName(q);
        rejectionFlags = rejFlags;
        magMagnitude = magMag;
        dmpHeadingDeg = dmpHdg;
        dmpHeadingValid = dmpValid;
        compassHeadingDeg = compassHdg;
        fusionHeadingDeg = fusionHdg;
        fusionValid = fusOk;
        magState = mState;
        headingHoldover = holdover;
        headingHoldoverState = headingHoldoverStateName(holdoverState);
    }
    void setMagRaw(const Vec3 &raw, const Vec3 &boat, const Vec3 &corrected)
    {
        GWSYNCHRONIZED(lock);
        magRawX = raw.x;
        magRawY = raw.y;
        magRawZ = raw.z;
        magBoatX = boat.x;
        magBoatY = boat.y;
        magBoatZ = boat.z;
        magCorrX = corrected.x;
        magCorrY = corrected.y;
        magCorrZ = corrected.z;
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
        doc["dmpActive"] = dmpActive;
        doc["rot"] = rotDegPerSec;
        doc["rotDerived"] = rotDerivedDegPerSec;
        doc["rotDisagrees"] = rotDisagrees;
        doc["headingSource"] = headingSource;
        doc["headingQuality"] = headingQuality;
        doc["rejectionFlags"] = rejectionFlags;
        doc["magMagnitude"] = magMagnitude;
        doc["dmpHeading"] = dmpHeadingDeg;
        doc["dmpHeadingValid"] = dmpHeadingValid;
        doc["compassHeading"] = compassHeadingDeg;
        doc["fusionHeading"] = fusionHeadingDeg;
        doc["fusionValid"] = fusionValid;
        doc["magDisturbed"] = (magState == MagDisturbanceState::Disturbed);
        doc["headingHoldover"] = headingHoldover;
        doc["headingHoldoverState"] = headingHoldoverState;
        doc["magRawX"] = magRawX;
        doc["magRawY"] = magRawY;
        doc["magRawZ"] = magRawZ;
        doc["magBoatX"] = magBoatX;
        doc["magBoatY"] = magBoatY;
        doc["magBoatZ"] = magBoatZ;
        doc["magCorrX"] = magCorrX;
        doc["magCorrY"] = magCorrY;
        doc["magCorrZ"] = magCorrZ;
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
    uint32_t fifoFramesDrained = 0, fifoOverflows = 0;
    uint32_t freeHeapBytes = 0, minFreeHeapEverBytes = 0;
    uint32_t stackHighWaterMarkBytes = 0;

public:
    Icm20948PerfStats() { lock = xSemaphoreCreateMutex(); }
    ~Icm20948PerfStats() { vSemaphoreDelete(lock); }

    void update(uint32_t sensorUs, uint32_t procUs, uint32_t fusUs, uint32_t logUs, uint32_t nmeaUs,
                uint32_t totalUs, bool missedDeadline, uint32_t framesDrained, uint32_t overflows,
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
        fifoFramesDrained = framesDrained;
        fifoOverflows = overflows;
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
        doc["fifoFramesDrained"] = fifoFramesDrained;
        doc["fifoOverflows"] = fifoOverflows;
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

// A cheap "where in the loop was I last" marker, updated at each key step
// below with a plain string-literal pointer assignment (no formatting, no
// logging - negligible cost every cycle). Read by the watchdog only in
// the rare case it's about to force a restart, so a restart's log message
// says exactly which call the task was in rather than just that it froze
// - see doc/IcmMagnetometerDmpConflict.md's "Bench verification" section
// for the real hang this helped root-cause live.
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
        GwJsonDocument doc(640);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    Icm20948PerfStats perfStats;
    api->registerRequestHandler("perfStatus", [&perfStats](AsyncWebServerRequest *request)
                                 {
        GwJsonDocument doc(384);
        perfStats.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    // Registers capControl/capStatus/capDownload - must run from here
    // (see capture's declaration comment above), NOT from initIcm20948
    // (where the writer task itself was already started).
    capture.begin(api);

    GwIcm20948HardwareAdapter hw;
    if (!hw.begin(logger, GWICM20948_SDA_PIN, GWICM20948_SCL_PIN))
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found - task stopped");
        vTaskDelete(NULL);
        return;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found, starting attitude updates");
    GwConfigHandler *config = api->getConfig();

    Icm20948CalControl calControl;
    calControl.begin(api, config);

    int accRangeIdx = 0, gyrRangeIdx = 0;
    config->getValue(accRangeIdx, GwConfigDefinitions::icmAccRange, 0);
    config->getValue(gyrRangeIdx, GwConfigDefinitions::icmGyrRange, 0);
    hw.setFullScale(logger, accRangeIdx, gyrRangeIdx);

    bool useDmp = true;
    config->getValue(useDmp, GwConfigDefinitions::icmUseDmp, true);
    bool dmpOk = useDmp && hw.initDmp(logger);
    webData.setDmpActive(dmpOk);

    int rateHz = 10;
    config->getValue(rateHz, GwConfigDefinitions::icmRateHz, 10);
    if (rateHz < 1)
        rateHz = 1;
    unsigned long loopDelayMs = 1000UL / (unsigned long)rateHz;
    LOG_DEBUG(GwLog::LOG, "ICM20948 update rate %d Hz (delay %lu ms)", rateHz, loopDelayMs);

    unsigned char sid = 0;
    unsigned long taskStartMs = millis();
    unsigned long lastCycleMs = millis();

    bool haveDmpSample = false;
    Quaternion lastDmpQuat;
    unsigned long lastDmpSampleMs = 0;

    // Persists the last good DMP-sourced compass reading the same way
    // lastDmpQuat/haveDmpSample persist the last good quaternion - the
    // compass field doesn't necessarily land in every single frame this
    // cycle drains, so "sticky last value" (not "this cycle only") is the
    // right semantics. See doc/IcmMagnetometerDmpConflict.md.
    bool haveDmpCompassSample = false;
    Vec3 lastDmpMagRaw;

    // The actual heading/attitude pipeline (fusion filter, DMP validator,
    // source selector, mag disturbance monitor, heading filter, ROT
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

    // Arm the watchdog only now - startup above (I2C scan, up to 6 sensor
    // init retries at 300ms each, DMP init) can legitimately take a few
    // seconds and shouldn't be mistaken for a hang.
    g_icm20948LastHeartbeatMs = millis();
    g_icm20948WatchdogArmed = true;

    while (true)
    {
        g_icm20948LastCheckpoint = "before dataReady";
        delay(loopDelayMs);
        if (!hw.dataReady())
            continue;
        // Real bug found live, 2026-08-06: this used to refresh the
        // heartbeat unconditionally at the very top of the loop, before
        // even checking dataReady(). That made the watchdog blind to
        // exactly the failure mode it needed to catch most: dataReady()
        // returning false forever (not hanging, just perpetually "no new
        // data") lets the loop spin harmlessly through delay()+continue
        // indefinitely, refreshing a heartbeat that looks perfectly fresh
        // while roll/pitch/heading never update again - confirmed live via
        // a real-time watchdog check log that sat at "checkpoint=before
        // dataReady" with heartbeat age never exceeding ~100ms for 90+
        // seconds straight during a real hang. The heartbeat now only
        // advances on a cycle that actually got past dataReady(), so it
        // reflects genuine progress - at the configured rate (up to
        // 10Hz+) dataReady() should return true well within the 8s
        // timeout under any normal operation, so this doesn't cost any
        // real margin against the original hang scenario this watchdog
        // was built for either.
        g_icm20948LastHeartbeatMs = millis();
        g_icm20948LastCheckpoint = "after dataReady, before readAGMT";

        unsigned long loopStartUs = micros();
        unsigned long sensorReadUs = 0;
        unsigned long nmeaSendUs = 0;

        unsigned long sensorReadStartUs = micros();
        hw.readAGMT();
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

        String headingModeStr = "dmp";
        config->getValue(headingModeStr, GwConfigDefinitions::icmHeadingMode, "dmp");
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
        Vec3 magBoat = ImuCoordinateTransform::toBoatFrame(hw.readMagRaw(), orientation);
        sensorReadUs += micros() - sensorReadStartUs;
        webData.setAccel(accelBoat.x, accelBoat.y, accelBoat.z);

        // --- DMP sample (if enabled/available). Placed here, before
        // anything below consumes magBoat (calibration engines, cycleIn),
        // so the magBoat override a few lines down is visible to every
        // downstream consumer this cycle - not just the ones that happen
        // to run later in the function. ---
        unsigned long dmpReadStartUs = micros();
        bool dmpFreshThisCycle = false;
        if (dmpOk)
        {
            g_icm20948LastCheckpoint = "before readDmpQuaternion";
            Quaternion q;
            if (hw.readDmpQuaternion(q))
            {
                lastDmpQuat = q;
                haveDmpSample = true;
                lastDmpSampleMs = nowMs;
                dmpFreshThisCycle = true;
            }
            g_icm20948LastCheckpoint = "before readDmpCompass";
            // Must run after readDmpQuaternion() above - see readDmpCompass()'s
            // doc comment (it reads back what that call just captured, it
            // does not drain the FIFO itself).
            Vec3 dmpMag;
            if (hw.readDmpCompass(dmpMag))
            {
                lastDmpMagRaw = dmpMag;
                haveDmpCompassSample = true;
            }
            g_icm20948LastCheckpoint = "after DMP block";
        }
        sensorReadUs += micros() - dmpReadStartUs;
        int dmpAgeMs = (int)(nowMs - lastDmpSampleMs);

        // The non-DMP raw-register magnetometer parse (readMagRaw(), used
        // for magBoat above) decodes garbage once DMP is active - DMP's own
        // startupDMP() reconfigures I2C_SLV0's shadow-register layout for
        // its own use, which readAGMT()'s fixed-layout parsing doesn't know
        // about. See doc/IcmMagnetometerDmpConflict.md. Once DMP is active
        // and has produced at least one good compass sample, prefer that
        // DMP-native source instead - falls back to the raw parse only
        // while DMP is off or hasn't produced a compass sample yet (e.g.
        // briefly at boot), where the raw parse is the correct path. Done
        // before the calibration engines below are fed, so both the live
        // Calibration-panel hard-iron tracking (calControl.feedMagSample)
        // and the CSV diagnostic capture see the corrected value too, not
        // just the heading pipeline.
        if (dmpOk && haveDmpCompassSample)
            magBoat = ImuCoordinateTransform::toBoatFrame(lastDmpMagRaw, orientation);

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
        cycleIn.dmpOk = dmpOk;
        cycleIn.haveDmpSample = haveDmpSample;
        cycleIn.dmpFreshThisCycle = dmpFreshThisCycle;
        // DMP computes this quaternion entirely in the chip's own fixed
        // physical frame - it has no idea icmOrientation exists. Rotate it
        // into boat frame here, at the same point accelBoat/gyroBoat/
        // magBoat already get transformed above, so DMP-sourced roll/
        // pitch/heading respect the configured mounting orientation
        // instead of silently bypassing it (real bug, found bench-testing:
        // switching orientation had zero effect on DMP-mode roll/pitch).
        cycleIn.dmpQuat = ImuCoordinateTransform::rotateDmpQuaternion(lastDmpQuat, orientation);
        cycleIn.dmpAgeMs = (unsigned long)dmpAgeMs;
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

        webData.setMagRaw(hw.readMagRaw(), magBoat, cycleOut.magCorrected);

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
                                cycleOut.rawDmpHeadingDeg, cycleOut.dmpCandidateValid, cycleOut.rawCompassHeadingDeg, cycleOut.rawFusionHeadingDeg,
                                cycleOut.fusionCandidateValid, cycleOut.magDisturbanceState, cycleOut.headingHoldover, cycleOut.headingHoldoverState);

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
            sample.magRaw = hw.readMagRaw();
            sample.accelBoat = accelBoat;
            sample.gyroBoat = gyroBoat;
            sample.magBoat = magBoat;
            sample.magCorrected = cycleOut.magCorrected;
            sample.magMagnitude = cycleOut.magMagnitude;
            sample.dmpQ0 = lastDmpQuat.w;
            sample.dmpQ1 = lastDmpQuat.x;
            sample.dmpQ2 = lastDmpQuat.y;
            sample.dmpQ3 = lastDmpQuat.z;
            sample.dmpRollDeg = cycleOut.dmpRollDeg;
            sample.dmpPitchDeg = cycleOut.dmpPitchDeg;
            sample.dmpHeadingDeg = cycleOut.rawDmpHeadingDeg;
            sample.compassHeadingDeg = cycleOut.rawCompassHeadingDeg;
            sample.fusionHeadingDeg = cycleOut.rawFusionHeadingDeg;
            sample.outputHeadingDeg = cycleOut.headingValid ? cycleOut.headingDeg : -1.0;
            sample.rateOfTurnDegPerSec = cycleOut.rotDegPerSec;
            sample.activeSource = cycleOut.headingSource;
            sample.headingQuality = cycleOut.headingQuality;
            sample.rejectionFlags = cycleOut.rejectionFlags;
            sample.dmpSampleAgeMs = dmpAgeMs;
            sample.fifoErrorCount = hw.fifoErrorCount();
            sample.sensorErrorCount = 0; // no lower-level read-failure signal exposed by this hardware adapter/library yet
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
                          totalLoopUs, missedDeadline, hw.fifoFramesDrained(), hw.fifoErrorCount(),
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
    // confirmed created and running - see
    // doc/IcmMagnetometerDmpConflict.md's "Bench verification" section).
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
