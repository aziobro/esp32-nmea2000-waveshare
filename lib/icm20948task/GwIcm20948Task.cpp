#include "GwIcm20948Task.h"
#include "GwHardware.h"
#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"
#include <N2kMessages.h>

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
                         MagDisturbanceState mState)
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

static void runIcm20948Task(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "icm20948 task starting, sda=%d scl=%d", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);

    Icm20948WebData webData;
    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                {
        GwJsonDocument doc(640);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    Icm20948Capture capture;
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

    while (true)
    {
        delay(loopDelayMs);
        if (!hw.dataReady())
            continue;
        hw.readAGMT();

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
        webData.setAccel(accelBoat.x, accelBoat.y, accelBoat.z);

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
        ImuCalibration cal = cachedCal;
        Vec3 gyroCal = ImuCalibrationOps::applyGyro(gyroBoat, cal);

        // --- Feed the interactive calibration engines (Calibration/
        // Gyroscope web panels) - cheap, no-ops unless a swing/stationary
        // capture is actively running. Uses the RAW (pre-calibration)
        // gyro/mag boat-frame vectors, since these engines measure
        // absolute bias/scale, not a residual on top of whatever's
        // already applied - see GwIcm20948CalControlTask.h. ---
        calControl.feedGyroSample(gyroBoat, accelBoat.norm());
        calControl.feedMagSample(magBoat.x, magBoat.y);

        // --- DMP sample (if enabled/available) ---
        bool dmpFreshThisCycle = false;
        if (dmpOk)
        {
            Quaternion q;
            if (hw.readDmpQuaternion(q))
            {
                lastDmpQuat = q;
                haveDmpSample = true;
                lastDmpSampleMs = nowMs;
                dmpFreshThisCycle = true;
            }
        }
        int dmpAgeMs = (int)(nowMs - lastDmpSampleMs);

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
        cycleIn.dmpQuat = lastDmpQuat;
        cycleIn.dmpAgeMs = (unsigned long)dmpAgeMs;
        cycleIn.dtSec = dtSec;
        cycleIn.nowMs = nowMs;
        cycleIn.taskStartMs = taskStartMs;
        cycleIn.cal = cal;
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

        ImuCycleOutput cycleOut = cycleProcessor.process(cycleIn);

        webData.setRot(cycleOut.rotDegPerSec);
        if (sendRot)
        {
            tN2kMsg rotMsg;
            SetN2kRateOfTurn(rotMsg, sid, radians(cycleOut.rotDegPerSec));
            api->sendN2kMessage(rotMsg);
            sid = (sid + 1) % 252;
        }

        webData.setMagRaw(hw.readMagRaw(), magBoat, cycleOut.magCorrected);

        if (cycleOut.attitudeValid)
        {
            webData.set(cycleOut.rollDeg, cycleOut.pitchDeg, cycleOut.rawRollDeg, cycleOut.rawPitchDeg);

            api->setCalibrationValue(GwConfigDefinitions::icmRollOff, cycleOut.rawRollDeg);
            api->setCalibrationValue(GwConfigDefinitions::icmPitchOff, cycleOut.rawPitchDeg);

            if (sendAttitude)
            {
                tN2kMsg msg;
                SetN2kAttitude(msg, sid, haveLastHeading ? lastHeadingRad : N2kDoubleNA, radians(cycleOut.pitchDeg), radians(cycleOut.rollDeg));
                api->sendN2kMessage(msg);
                sid = (sid + 1) % 252;
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

            if (sendHeading && headingMode != HeadingSourceMode::DiagnosticOnly)
            {
                tN2kMsg hdgMsg;
                SetN2kMagneticHeading(hdgMsg, sid, radians(cycleOut.headingDeg));
                api->sendN2kMessage(hdgMsg);
                sid = (sid + 1) % 252;
            }

            webData.setRotDiagnostic(cycleOut.rotDerivedDegPerSec, cycleOut.rotDisagrees);
        }
        else
        {
            webData.setHeadingInvalid();
        }

        webData.setDiagnostics(cycleOut.headingSource, cycleOut.headingQuality, cycleOut.rejectionFlags, cycleOut.magMagnitude,
                                cycleOut.rawDmpHeadingDeg, cycleOut.dmpCandidateValid, cycleOut.rawCompassHeadingDeg, cycleOut.rawFusionHeadingDeg,
                                cycleOut.fusionCandidateValid, cycleOut.magDisturbanceState);

        // --- Diagnostic CSV capture (Phase 2) - offerSample() is a
        // cheap, non-blocking queue push that no-ops entirely unless
        // capture or serial-CSV output is actually active, so this costs
        // nothing in the common case. ---
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
    // Stack is 16000 bytes - unchanged from the pre-rewrite code (the
    // intermittent cold-boot crash this project hit was an I2C/WiFi boot
    // race, not stack exhaustion - see GwIcm20948HardwareAdapter::begin()).
    api->addUserTask(runIcm20948Task, String("icm20948Task"), 16000);
}
