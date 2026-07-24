#include "GwIcm20948Task.h"
#include "GwHardware.h"
#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"
#include <N2kMessages.h>
#include <ICM_20948.h>

/*
  Attitude (PGN 127257, roll/pitch from the accelerometer) is geometrically
  valid as soon as the axis-to-boat mapping is right, which
  icmRollInv/icmPitchInv/icmRollOff/icmPitchOff (icm20948Config.json,
  category "icm20948") correct for.

  Heading (PGN 127250, from the magnetometer) needed real calibration before
  it was safe to send at all - see the hard-iron tracking below. Even
  calibrated, this is a SIMPLE hard-iron-only (constant offset per axis)
  correction, not a full soft-iron (ellipsoid) calibration - a proper compass
  swing builds a deviation table across many headings, which this doesn't
  attempt. Expect a few degrees of residual, heading-dependent error,
  especially near other electronics/metal. Good enough for a rough magnetic
  heading, not for precision navigation.
*/

#ifndef GWICM20948_SDA_PIN
#define GWICM20948_SDA_PIN -1
#endif
#ifndef GWICM20948_SCL_PIN
#define GWICM20948_SCL_PIN -1
#endif

// Normalizes to (-180,180] - without this, raw+offset can land outside the
// expected range (e.g. 359 instead of -1) whenever the sum crosses the
// wraparound boundary. Note this alone does NOT fix a 180°-mounting-flip
// correction - that needs a sign inversion (icmRollInv/icmPitchInv), not an
// additive offset; the two are different operations and one can't
// substitute for the other, this only makes the offset math well-behaved.
static double wrapDeg180(double deg)
{
    deg = fmod(deg + 180.0, 360.0);
    if (deg < 0)
        deg += 360.0;
    return deg - 180.0;
}

// Normalizes to [0,360) - headings are conventionally 0..360, not -180..180.
static double wrapDeg360(double deg)
{
    deg = fmod(deg, 360.0);
    if (deg < 0)
        deg += 360.0;
    return deg;
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
    double heading = 0;               // calibrated magnetic heading, degrees
    bool headingValid = false;
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
    }
};

static void runIcm20948Task(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "icm20948 task starting, sda=%d scl=%d", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
    Wire.setTimeOut(1000); // bound any I2C hang instead of blocking this task forever
    bool wireOk = Wire.begin(GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
    Wire.setClock(400000);
    LOG_DEBUG(GwLog::LOG, "Wire.begin returned %d", (int)wireOk);

    LOG_DEBUG(GwLog::LOG, "scanning I2C bus...");
    int devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            LOG_DEBUG(GwLog::LOG, "  I2C device found at 0x%02X", addr);
            devicesFound++;
        }
    }
    LOG_DEBUG(GwLog::LOG, "I2C scan done, %d device(s) found", devicesFound);

    Icm20948WebData webData;
    // Register the endpoint before we know whether the sensor was found, so
    // the web UI's IMU tab gets a clean "no data" (valid:false) instead of a
    // 404 while we're still retrying / if the sensor is missing.
    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                {
        GwJsonDocument doc(256);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    ICM_20948_I2C imu;
    bool found = false;
    // AD0_VAL=1 -> 0x69 (default, ADR jumper open), AD0_VAL=0 -> 0x68 (jumper closed)
    for (int attempt = 0; attempt < 6 && !found; attempt++)
    {
        uint8_t ad0 = (attempt % 2 == 0) ? 1 : 0;
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) starting", attempt, ad0);
        imu.begin(Wire, ad0);
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) result: %s", attempt, ad0, imu.statusString());
        if (imu.status == ICM_20948_Stat_Ok)
        {
            found = true;
        }
        else
        {
            delay(300);
        }
    }
    if (!found)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found on sda=%d,scl=%d - task stopped", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
        vTaskDelete(NULL);
        return;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found, starting attitude updates");
    GwConfigHandler *config = api->getConfig();
    unsigned char sid = 0;

    // Hard-iron tracking: running min/max of the raw magnetometer X/Y since
    // this task started (a restart clears it - see icm20948Config.json for
    // why that's fine rather than needing an explicit reset control). The
    // midpoint of the swing on each axis is fed live to the Config page's
    // "C" button for icmMagXOff/icmMagYOff, same generic mechanism as the
    // roll/pitch offsets.
    double magXMin = 1e6, magXMax = -1e6, magYMin = 1e6, magYMax = -1e6;

    while (true)
    {
        delay(200);
        if (!imu.dataReady())
            continue;
        imu.getAGMT();
        double accX = imu.accX();
        double accY = imu.accY();
        double accZ = imu.accZ();
        double rawRollRad = atan2(accY, accZ);
        double rawPitchRad = atan2(-accX, sqrt(accY * accY + accZ * accZ));

        // Re-read every cycle so calibration changes on the Config page take
        // effect immediately, no reboot needed.
        float rollOffsetDeg = 0, pitchOffsetDeg = 0;
        bool rollInvert = false, pitchInvert = false;
        bool sendAttitude = true;
        config->getValue(rollOffsetDeg, GwConfigDefinitions::icmRollOff, 0.0f);
        config->getValue(pitchOffsetDeg, GwConfigDefinitions::icmPitchOff, 0.0f);
        config->getValue(rollInvert, GwConfigDefinitions::icmRollInv, false);
        config->getValue(pitchInvert, GwConfigDefinitions::icmPitchInv, false);
        config->getValue(sendAttitude, GwConfigDefinitions::icmSendAtt, true);

        // Invert (sign flip, for a 180° mounting error) happens before the
        // fine offset - "raw" below means "after mounting-orientation
        // correction, before fine-tuning", which is what the offset and the
        // Config page's calibrate button operate on.
        double rawRollDeg = rollInvert ? -degrees(rawRollRad) : degrees(rawRollRad);
        double rawPitchDeg = pitchInvert ? -degrees(rawPitchRad) : degrees(rawPitchRad);
        double rollDeg = wrapDeg180(rawRollDeg + rollOffsetDeg);
        double pitchDeg = wrapDeg180(rawPitchDeg + pitchOffsetDeg);
        webData.set(rollDeg, pitchDeg, rawRollDeg, rawPitchDeg);

        // Feeds the Config page's generic "C" (calibrate) button/dialog for
        // icmRollOff/icmPitchOff (see icm20948Config.json, type "calval") -
        // same generic /api/calibrate mechanism lib/spitask's DMS22B zero
        // calibration uses. The config item's "eval":"-v" negates this raw
        // value to preview the offset that would zero it out.
        api->setCalibrationValue(GwConfigDefinitions::icmRollOff, rawRollDeg);
        api->setCalibrationValue(GwConfigDefinitions::icmPitchOff, rawPitchDeg);

        LOG_DEBUG(GwLog::DEBUG, "ICM20948 roll=%.1f pitch=%.1f (deg, calibrated)", rollDeg, pitchDeg);
        if (sendAttitude)
        {
            tN2kMsg msg;
            SetN2kAttitude(msg, sid, N2kDoubleNA, radians(pitchDeg), radians(rollDeg));
            api->sendN2kMessage(msg);
            sid = (sid + 1) % 252;
        }

        // --- Compass (magnetometer) ---
        double magX = imu.magX();
        double magY = imu.magY();
        double magZ = imu.magZ();
        if (magX < magXMin)
            magXMin = magX;
        if (magX > magXMax)
            magXMax = magX;
        if (magY < magYMin)
            magYMin = magY;
        if (magY > magYMax)
            magYMax = magY;
        double magXMid = (magXMin + magXMax) / 2.0;
        double magYMid = (magYMin + magYMax) / 2.0;
        // Feeds icmMagXOff/icmMagYOff's "C" button - unlike roll/pitch these
        // are "calset" (plain copy), the field IS the offset to subtract,
        // no negation needed.
        api->setCalibrationValue(GwConfigDefinitions::icmMagXOff, magXMid);
        api->setCalibrationValue(GwConfigDefinitions::icmMagYOff, magYMid);

        float magXOffset = 0, magYOffset = 0, hdgOffsetDeg = 0;
        bool hdgInvert = false, sendHeading = false;
        config->getValue(magXOffset, GwConfigDefinitions::icmMagXOff, 0.0f);
        config->getValue(magYOffset, GwConfigDefinitions::icmMagYOff, 0.0f);
        config->getValue(hdgOffsetDeg, GwConfigDefinitions::icmHdgOff, 0.0f);
        config->getValue(hdgInvert, GwConfigDefinitions::icmHdgInv, false);
        config->getValue(sendHeading, GwConfigDefinitions::icmSendHdg, false);

        double mx = magX - magXOffset;
        double my = magY - magYOffset;
        // mz is not hard-iron corrected - a boat-mounted sensor can only be
        // rotated in yaw (about the vertical axis) during a swing, never
        // tumbled through enough 3D orientations to calibrate the Z axis
        // properly, so we don't pretend to.

        // Standard tilt-compensated compass formula, using the CALIBRATED
        // roll/pitch (the boat's actual attitude) to de-tilt the reading.
        double rollRad = radians(rollDeg);
        double pitchRad = radians(pitchDeg);
        double Xh = mx * cos(pitchRad) + my * sin(rollRad) * sin(pitchRad) + magZ * cos(rollRad) * sin(pitchRad);
        double Yh = my * cos(rollRad) - magZ * sin(rollRad);
        double rawHeadingDeg = degrees(atan2(Yh, Xh));
        // Whether this needs Invert depends on the chip's axis handedness,
        // which isn't knowable from code - same as the roll/pitch case,
        // check empirically (point the bow at a known heading) and flip
        // icmHdgInv if it runs backwards (e.g. increases turning left).
        if (hdgInvert)
            rawHeadingDeg = -rawHeadingDeg;
        double headingDeg = wrapDeg360(rawHeadingDeg + hdgOffsetDeg);
        webData.setHeading(headingDeg);
        // icmHdgOff is "calval" with eval "-v", same convention as roll/pitch.
        api->setCalibrationValue(GwConfigDefinitions::icmHdgOff, rawHeadingDeg);

        LOG_DEBUG(GwLog::DEBUG, "ICM20948 heading=%.1f (deg, calibrated), magMid=(%.1f,%.1f)", headingDeg, magXMid, magYMid);
        if (sendHeading)
        {
            tN2kMsg hdgMsg;
            SetN2kMagneticHeading(hdgMsg, sid, radians(headingDeg));
            api->sendN2kMessage(hdgMsg);
            sid = (sid + 1) % 252;
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
    api->addUserTask(runIcm20948Task, String("icm20948Task"), 4000);
}
