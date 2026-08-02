#pragma once
#include "ImuTypes.h"
#include "ImuAngleMath.h"

/*
  Stationary gyroscope bias calibration - a pure state machine (no
  hardware I/O; the task layer feeds it samples and reads results back).
  Requires the boat to be still (low gyro motion, accel magnitude near
  1g) so any nonzero average gyro reading over the sample window can only
  be sensor bias, not real rotation.

  Also home to the rate-of-turn helper functions (Phase 12): a plain
  (non-circular - rate of turn is a rate, not an angle) low-pass filter,
  and comparing the direct gyro-Z-based ROT estimate against the
  derivative of the fused/unwrapped heading, as a plausibility cross-check.
*/

enum class GyroCalState
{
    Idle,
    Collecting,
    Done,
    Failed
};

struct GyroCalConfig
{
    // All provisional - see doc/IcmHeadingArchitecture.md; real values
    // need tuning against the actual sensor's noise characteristics.
    int requiredSamples = 200;
    double maxGyroMagDegPerSec = 2.0;  // "low motion" gate
    double accelMagToleranceG = 0.05;  // how close to 1g counts as "stationary/level"
    double maxStdDevDegPerSec = 0.5;   // reject an unstable/noisy calibration
};

class GyroCalEngine
{
public:
    void start();
    void cancel();

    // Feeds one sample. Returns false if the sample was rejected (motion
    // detected, or accel magnitude too far from 1g) - rejected samples
    // don't count toward requiredSamples and don't affect the running
    // statistics, but do NOT reset progress (a brief bump doesn't throw
    // away otherwise-good progress, matching how the 2D mag calibration
    // engine handles this too).
    bool addSample(const Vec3 &gyroDegPerSec, double accelMagG, const GyroCalConfig &cfg = GyroCalConfig());

    GyroCalState state() const { return st; }
    int sampleCount() const { return count; }

    // Valid only once state()==Done.
    Vec3 resultBiasDegPerSec() const { return mean; }
    Vec3 resultStdDevDegPerSec() const;

private:
    GyroCalState st = GyroCalState::Idle;
    int count = 0;
    Vec3 mean;
    Vec3 m2; // Welford's algorithm running sum of squared differences from the mean, per axis
};

namespace ImuRateOfTurn
{
    // Ordinary (non-circular) exponential low-pass - rate of turn is a
    // rate, not a wrapping angle, so ImuAngleMath's circular filters don't
    // apply here.
    double lowPass(double prevDegPerSec, double newDegPerSec, double alpha);

    // Cross-checks the direct gyro-based ROT estimate against the
    // derivative of an unwrapped heading accumulator over dtSec - large,
    // persistent disagreement suggests a bad gyro reading or bias, not
    // used to gate output here (that's an orchestration-layer decision)
    // but exposed as a testable, reusable comparison.
    double derivedFromHeadingDegPerSec(double headingUnwrappedDegNow, double headingUnwrappedDegPrev, double dtSec);

    // True if the two estimates disagree by more than toleranceDegPerSec.
    bool disagreesWithHeadingDerivative(double gyroRotDegPerSec, double headingDerivedDegPerSec, double toleranceDegPerSec);
}
