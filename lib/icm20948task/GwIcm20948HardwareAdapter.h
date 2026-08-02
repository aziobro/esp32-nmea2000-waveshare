#pragma once
#include "ImuTypes.h"
#include <ICM_20948.h>
#include "GwLog.h"

/*
  Thin wrapper around the ICM_20948 library - the only place in this
  task's own directory that touches the chip directly. Everything it
  hands back (Vec3/Quaternion) feeds straight into the pure pipeline in
  lib/icm20948pure. Owns the I2C bring-up, sensor detection retry loop,
  and DMP init sequence exactly as before (unchanged from the pre-rewrite
  GwIcm20948Task.cpp - only moved).
*/
class GwIcm20948HardwareAdapter
{
public:
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
    void readAGMT(); // must be called once per cycle before the read*() functions below

    Vec3 readAccelG() const;      // g
    Vec3 readGyroDegPerSec() const; // degrees/second
    Vec3 readMagRaw() const;      // raw magnetometer units

    // Drains the FIFO fully (more than one frame can be queued if the
    // DMP's internal rate exceeds our poll rate - only the newest
    // matters). Returns true if at least one new Quat9 sample was found.
    bool readDmpQuaternion(Quaternion &out);

    // Count of readDMPdataFromFIFO() calls that returned an error status
    // other than Ok/FIFOMoreDataAvail, since boot - exposed for
    // diagnostics (CSV capture, web tab), not used for any pipeline
    // decision itself.
    uint32_t fifoErrorCount() const { return fifoErrors; }

    // Count of FIFO frames actually drained with a successfully-parsed
    // Quat9 sample, since boot - the Performance panel's "FIFO frames
    // drained" counter (see doc/IcmPerformanceReview.md).
    uint32_t fifoFramesDrained() const { return framesDrained; }

private:
    ICM_20948_I2C imu;
    double lastAccX = 0, lastAccY = 0, lastAccZ = 0;
    double lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0;
    double lastMagX = 0, lastMagY = 0, lastMagZ = 0;
    uint32_t fifoErrors = 0;
    uint32_t framesDrained = 0;
};
