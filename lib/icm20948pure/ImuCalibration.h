#pragma once
#include "ImuTypes.h"

/*
  Versioned sensor calibration model:

    corrected = A * (raw - bias)

  where bias is a per-axis offset and A is a 3x3 matrix (identity for a
  hard-iron-only correction, a full matrix once a real 3D/ellipsoid fit is
  available - see tools/imu_calibration/fit_calibration.py). Applied to the
  magnetometer (full matrix+bias) and, separately, to the accelerometer
  (diagonal scale+bias, no cross-axis terms - a plain scale/offset per axis
  is the standard accelerometer calibration model, a full ellipsoid fit
  isn't needed the way it is for magnetometers) and gyroscope (bias only,
  see ImuGyroCal).

  Identity/zero-bias is always the safe default - calibrating changes these
  values, never the other way around. No ArduinoJson/Arduino dependency
  here on purpose (this file compiles under the native test env); the task
  layer (GwIcm20948Task.cpp) is responsible for mapping these fields
  to/from the config system and the web UI's JSON.
*/

struct ImuCalibration
{
    uint32_t version = 1;

    double magBias[3] = {0, 0, 0};
    double magMatrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    double accelBias[3] = {0, 0, 0};
    double accelScale[3] = {1, 1, 1};

    double gyroBias[3] = {0, 0, 0};

    double fixedHeadingOffsetDeg = 0;

    bool magCalibrationValid = false;
    bool accelCalibrationValid = false;
    bool gyroCalibrationValid = false;

    // 0 (no confidence) to 1 (excellent) - set by whichever calibration
    // engine produced this (see ImuMagCal2D), not computed here.
    double magCalibrationQuality = 0;

    // Bumped every time a calibration is saved - lets diagnostics/logs
    // show "this is the 3rd calibration since the unit was set up"
    // without needing a timestamp (this device has no RTC).
    uint32_t calibrationSequence = 0;
};

namespace ImuCalibrationOps
{
    // corrected = magMatrix * (raw - magBias)
    Vec3 applyMag(const Vec3 &raw, const ImuCalibration &cal);

    // corrected[i] = (raw[i] - accelBias[i]) * accelScale[i]
    Vec3 applyAccel(const Vec3 &raw, const ImuCalibration &cal);

    // corrected = raw - gyroBias
    Vec3 applyGyro(const Vec3 &raw, const ImuCalibration &cal);

    // Safe, identity/zero starting point - explicit rather than relying on
    // ImuCalibration's own default member initializers, so callers that
    // want to reset back to safe defaults have a clear single entry point.
    ImuCalibration identityDefault();

    // Reproduces the pre-rewrite code's hard-iron-only compass calibration
    // exactly, so the currently-deployed unit's calibrated output doesn't
    // change: magBias=(legacyMagXOff, legacyMagYOff, 0), magMatrix=identity
    // (Z was never hard-iron corrected before either - a boat-mounted
    // sensor can only swing in yaw, never tumbled through enough 3D
    // orientations to calibrate that axis), fixedHeadingOffsetDeg carries
    // over icmHdgOff directly.
    ImuCalibration migrateFromLegacy(double legacyMagXOff, double legacyMagYOff, double legacyHeadingOffsetDeg);

    // True if magMatrix is (near enough) the identity and magBias is (near
    // enough) zero - i.e. no real calibration has been applied yet.
    bool isMagIdentity(const ImuCalibration &cal, double tolerance = 1e-9);
}
