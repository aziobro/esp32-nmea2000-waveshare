#pragma once
#include "ImuTypes.h"
#if !defined(GWICM20948_USE_WOLLEWALD)
#include <ICM_20948.h>
#endif
#include "GwLog.h"

/*
  Thin wrapper around the selected ICM-20948 library - the only place in this
  task's own directory that touches the chip directly. Everything it
  hands back (Vec3/Quaternion) feeds straight into the pure pipeline in
  lib/icm20948pure. Owns the I2C bring-up, sensor detection retry loop,
  and, for the SparkFun backend, the DMP init sequence exactly as before.
*/
class GwIcm20948HardwareAdapter
{
public:
    struct DmpReadResult
    {
        bool haveQuaternion = false;
        bool haveCompass = false;
        bool fifoNoData = false;
        bool fifoError = false;
        bool drainLimitHit = false;
        uint8_t framesRead = 0;
    };

    // Runs the full bring-up sequence: the established startup delay
    // (I2C/WiFi boot-race workaround), Wire.begin(), an I2C bus scan (for
    // diagnostics only), then repeated begin() attempts against both
    // possible AD0 addresses. Returns false if the sensor was never
    // found - callers should stop the task in that case, same as before.
    bool begin(GwLog *logger, int sdaPin, int sclPin);

    // Full-scale range - a one-time hardware setting applied once, same
    // as the pre-rewrite code (changing it requires a restart, handled
    // by the config system's own save-restarts-device behavior).
    void setFullScale(GwLog *logger, int accelRangeIdx, int gyrRangeIdx);

    // Attempts DMP init; returns whether it succeeded. If false, the task
    // should fall back to the plain accel/gyro/mag path - a working
    // degraded mode beats a dead one, same as before.
    bool initDmp(GwLog *logger);

    bool dataReady();
    bool readAGMT(); // must be called once per cycle before the read*() functions below

    Vec3 readAccelG() const;      // g
    Vec3 readGyroDegPerSec() const; // degrees/second
    Vec3 readMagRaw() const;      // raw magnetometer units
    bool magValid() const { return lastMagValid; }

    // Drains the FIFO fully (more than one frame can be queued if the
    // DMP's internal rate exceeds our poll rate - only the newest
    // matters). Returns true if at least one new Quat9 sample was found.
    // As a side effect, also captures any Compass_Calibr field present in
    // the same frames into a private slot - see readDmpCompass() below for
    // why this isn't a second, independent FIFO drain.
    bool readDmpQuaternion(Quaternion &out);

    // Returns the DMP's own calibrated-compass reading (uT, same unit as
    // readMagRaw()) captured during the MOST RECENT readDmpQuaternion()
    // call - does NOT read the FIFO itself. Must be called only after
    // readDmpQuaternion() in the same cycle. This is deliberately not a
    // second independent FIFO drain: one readDMPdataFromFIFO() call
    // returns a single frame whose header can carry multiple fields at
    // once (Quat9 and Compass_Calibr routinely land in the very same
    // frame here), and readDmpQuaternion() already drains the FIFO fully
    // each cycle - a second do-while loop calling readDMPdataFromFIFO()
    // again afterwards would find nothing left to read and report false
    // every cycle. See doc/IcmMagnetometerDmpConflict.md: this exists
    // because the non-DMP raw-register magnetometer parsing in
    // readAGMT()/readMagRaw() silently decodes garbage once the DMP has
    // reconfigured I2C_SLV0's shadow-register layout - the DMP's own
    // Compass_Calibr FIFO field is the correct source once DMP is active.
    bool readDmpCompass(Vec3 &out);

    // Lightweight recovery helpers used by the task loop before the
    // external watchdog escalates to a whole-device restart.
    bool resetFifo(GwLog *logger);
    bool reinitialize(GwLog *logger, int sdaPin, int sclPin);
#if defined(GWICM20948_USE_WOLLEWALD)
    bool recoverMagBridge();
#else
    bool recoverMagBridge() { return false; }
#endif

    DmpReadResult lastDmpReadResult() const { return dmpReadResult; }

    // Count of readDMPdataFromFIFO() calls that returned an error status
    // other than Ok/FIFOMoreDataAvail, since boot - exposed for
    // diagnostics (CSV capture, web tab), not used for any pipeline
    // decision itself.
    uint32_t fifoErrorCount() const { return fifoErrors; }

    // Count of FIFO frames actually drained with a successfully-parsed
    // Quat9 sample, since boot - the Performance panel's "FIFO frames
    // drained" counter (see doc/IcmPerformanceReview.md).
    uint32_t fifoFramesDrained() const { return framesDrained; }
    uint32_t fifoDrainLimitCount() const { return fifoDrainLimits; }
    uint32_t sensorErrorCount() const { return sensorErrors; }
    uint32_t i2cRecoveryCount() const { return i2cRecoveries; }
    uint32_t magRecoveryCount() const { return magRecoveries; }
    uint32_t magNoDataCount() const { return magNoData; }
    uint32_t magOverflowCount() const { return magOverflows; }
    uint32_t magZeroCount() const { return magZeroSamples; }
    uint32_t i2cRetryCount() const { return i2cRetries; }
    const char *lastStatusString()
    {
#if defined(GWICM20948_USE_WOLLEWALD)
        return lastStatus;
#else
        return imu.statusString();
#endif
    }

private:
    static const uint8_t DMP_MAX_FIFO_FRAMES_PER_CYCLE = 24;

    bool beginInternal(GwLog *logger, int sdaPin, int sclPin, bool startupDelay, bool scanBus);

#if defined(GWICM20948_USE_WOLLEWALD)
    uint8_t imuAddress = 0;
    char lastStatus[64] = "direct Wollewald backend not started";
    int activeSdaPin = -1;
    int activeSclPin = -1;
    bool beginWollewaldDirect(uint8_t address);
    bool resetWollewaldDirect();
    bool recoverWollewaldI2cBus(const char *reason);
    bool selectWollewaldBankStop(uint8_t bank);
    uint8_t endTransmissionWithRetry(uint8_t bank, uint8_t reg, const char *op, bool allowBusRecovery);
    bool writeWollewaldRegisterStop(uint8_t bank, uint8_t reg, uint8_t value);
    bool writeWollewaldRegister16Stop(uint8_t bank, uint8_t reg, uint16_t value);
    bool readWollewaldAgmtStop();
    bool readWollewaldRegistersStop(uint8_t bank, uint8_t reg, uint8_t *dst, uint8_t len);
    bool writeWollewaldAk09916Register8Stop(uint8_t reg, uint8_t value);
    bool readWollewaldAk09916Register8Stop(uint8_t reg, uint8_t &value);
    bool enableWollewaldMagDataReadStop(uint8_t reg, uint8_t bytes);
    bool waitWollewaldSlv4Stop(uint16_t timeoutMs);
    float wollewaldAccRangeFactor = 1.0f;
    float wollewaldGyrRangeFactor = 1.0f;
#else
    ICM_20948_I2C imu;
#endif
    double lastAccX = 0, lastAccY = 0, lastAccZ = 0;
    double lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0;
    double lastMagX = 0, lastMagY = 0, lastMagZ = 0;
    double prevAccX = 0, prevAccY = 0, prevAccZ = 0;
    double prevGyrX = 0, prevGyrY = 0, prevGyrZ = 0;
    double prevMagX = 0, prevMagY = 0, prevMagZ = 0;
    bool lastMagValid = false;
#if defined(GWICM20948_USE_WOLLEWALD)
    bool wollewaldMagStatusValid = false;
#endif
    bool havePreviousFrame = false;
    uint8_t repeatedFrameCount = 0;
    uint32_t fifoErrors = 0;
    uint32_t framesDrained = 0;
    uint32_t fifoDrainLimits = 0;
    uint32_t sensorErrors = 0;
    uint32_t i2cRecoveries = 0;
    uint32_t magRecoveries = 0;
    uint32_t magNoData = 0;
    uint32_t magOverflows = 0;
    uint32_t magZeroSamples = 0;
    uint32_t i2cRetries = 0;
    uint8_t magRailCounts[3] = {0, 0, 0};
    Vec3 lastDmpCompassRaw;
    bool haveDmpCompassSample = false;
    DmpReadResult dmpReadResult;
};
