#pragma once
#include "ImuTypes.h"

/*
  Software 9-axis sensor fusion: a Mahony-style complementary filter
  (gyro integration corrected toward the accelerometer/magnetometer
  reference directions via proportional-integral feedback on the
  quaternion error). Chosen over Madgwick's gradient-descent formulation
  because its structure (explicit error term, explicit P/I gains) is
  easier to reason about and unit-test deterministically; both are
  standard, widely-implemented AHRS algorithms and the project spec
  accepts either. Hand-implemented here (not a new library dependency) -
  small enough (~80 lines) to fully own and test.

  Inputs are boat-frame (post ImuCoordinateTransform), calibrated (post
  ImuCalibrationOps) accel/gyro/mag. Output quaternion follows the same
  convention as the DMP's own Quat9 and ImuQuaternion::toEuler (rotation
  from boat frame to the reference/level-north frame).
*/
class MahonyFusion
{
public:
    // kp: proportional gain (how fast the filter trusts accel/mag over
    // gyro integration - higher = faster correction but more susceptible
    // to short-term accel noise from boat motion). ki: integral gain
    // (corrects slow gyro bias over time - 0 disables it, a reasonable
    // default until real gyro bias characteristics are known from
    // hardware, see ImuGyroCal for the complementary explicit-calibration
    // approach).
    explicit MahonyFusion(double kp = 2.0, double ki = 0.0);

    // Full 9-axis update. gyroRadPerSec/accel/mag all boat-frame. accel
    // and mag are normalized internally - any consistent unit works. If
    // mag's norm is ~0 (dropout), falls back to accel-only (6-axis)
    // correction automatically rather than corrupting the estimate with a
    // garbage heading reference.
    void update(const Vec3 &gyroRadPerSec, const Vec3 &accel, const Vec3 &mag, double dtSec);

    // Accel-only (6-axis) update - no heading correction, yaw will drift
    // freely from gyro integration alone. Used internally when mag isn't
    // available; exposed directly too since a caller may want it
    // explicitly (e.g. testing gyro integration in isolation).
    void updateImuOnly(const Vec3 &gyroRadPerSec, const Vec3 &accel, double dtSec);

    Quaternion quaternion() const { return q; }
    void reset(const Quaternion &initial = Quaternion());

private:
    Quaternion q;
    double kp, ki;
    Vec3 integralFeedback;

    void applyCorrectedGyroStep(const Vec3 &gyroRadPerSec, const Vec3 &error, double dtSec);
};
