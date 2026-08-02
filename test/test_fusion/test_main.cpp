#include <unity.h>
#include <math.h>
#include "ImuFusion.h"
#include "ImuQuaternion.h"
#include "ImuSimulator.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

void test_quaternion_stays_normalized_through_updates(void)
{
    MahonyFusion f(2.0, 0.0);
    for (int i = 0; i < 200; i++)
    {
        f.update(Vec3(0.05, -0.02, 0.1), Vec3(0.1, 0.05, 0.98), Vec3(20, 3, 44), 0.02);
        TEST_ASSERT_TRUE(ImuQuaternion::isValidUnit(f.quaternion(), 1e-6));
    }
}

// Starting from identity (wrong attitude), feeding a CONSTANT, correct
// accel+mag reading for a known target attitude should converge the
// filter to that attitude - the core Mahony correction behavior.
void test_converges_to_known_attitude_from_identity(void)
{
    double targetHeadingDeg = 90.0, targetRollDeg = 10.0, targetPitchDeg = 5.0;
    SimulatorState s;
    s.headingDeg = targetHeadingDeg;
    s.rollDeg = targetRollDeg;
    s.pitchDeg = targetPitchDeg;
    SimulatedSample sample = ImuSimulator::generateSample(s);

    MahonyFusion f(5.0, 0.0); // higher gain for fast convergence in a static test
    for (int i = 0; i < 2000; i++)
        f.update(Vec3(0, 0, 0), sample.accelG, sample.magRaw, 0.02);

    double r, p, y;
    ImuQuaternion::toEuler(f.quaternion(), r, p, y);
    double rDeg = r * 180.0 / M_PI, pDeg = p * 180.0 / M_PI, yDeg = y * 180.0 / M_PI;

    TEST_ASSERT_DOUBLE_WITHIN(0.5, targetRollDeg, rDeg);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, targetPitchDeg, pDeg);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, fabs(ImuAngleMath::shortestDiff(targetHeadingDeg, yDeg)));
}

// Starting already converged, then tracking a full level 360-degree
// rotation via the simulator sample-by-sample (gyro + accel + mag all
// consistent with the same evolving true heading) - the filter's
// estimate should track the sweep, ending back near the start.
void test_tracks_level_360_rotation(void)
{
    double periodSec = 30.0;
    double dt = 0.02;

    SimulatorState s0 = ImuSimulator::Scenarios::level360Rotation(0.0, periodSec);
    SimulatedSample sample0 = ImuSimulator::generateSample(s0);
    MahonyFusion f(3.0, 0.0);
    // Pre-converge at t=0 before tracking starts, same reasoning as the
    // identity-convergence test - a fresh filter needs a moment to lock
    // on before its tracking accuracy means anything.
    for (int i = 0; i < 300; i++)
        f.update(Vec3(0, 0, 0), sample0.accelG, sample0.magRaw, dt);

    double t = 0;
    while (t < periodSec)
    {
        SimulatorState s = ImuSimulator::Scenarios::level360Rotation(t, periodSec);
        SimulatedSample sample = ImuSimulator::generateSample(s);
        Vec3 gyroRad(0, 0, s.rateOfTurnDegPerSec * M_PI / 180.0);
        f.update(gyroRad, sample.accelG, sample.magRaw, dt);
        t += dt;
    }

    double r, p, y;
    ImuQuaternion::toEuler(f.quaternion(), r, p, y);
    double yDeg = y * 180.0 / M_PI;
    // Back near the start (0/360) after a full rotation.
    TEST_ASSERT_DOUBLE_WITHIN(3.0, 0.0, fabs(ImuAngleMath::shortestDiff(0.0, yDeg)));
}

// Mag dropout (zero-norm reading) must not corrupt the estimate (NaN) and
// must fall back to accel-only tracking rather than applying a garbage
// heading correction.
void test_mag_dropout_falls_back_without_corrupting_state(void)
{
    SimulatorState s;
    s.headingDeg = 45.0;
    s.rollDeg = 5.0;
    SimulatedSample sample = ImuSimulator::generateSample(s);

    MahonyFusion f(3.0, 0.0);
    for (int i = 0; i < 200; i++)
        f.update(Vec3(0, 0, 0), sample.accelG, sample.magRaw, 0.02);

    // Now drop the mag out.
    for (int i = 0; i < 50; i++)
        f.update(Vec3(0.01, 0, 0), sample.accelG, Vec3(0, 0, 0), 0.02);

    TEST_ASSERT_TRUE(ImuQuaternion::isValidUnit(f.quaternion(), 1e-6));
    double r, p, y;
    ImuQuaternion::toEuler(f.quaternion(), r, p, y);
    TEST_ASSERT_TRUE(isfinite(r) && isfinite(p) && isfinite(y));
    // Roll should still be reasonably close (accel correction still active).
    TEST_ASSERT_DOUBLE_WITHIN(3.0, 5.0, r * 180.0 / M_PI);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_quaternion_stays_normalized_through_updates);
    RUN_TEST(test_converges_to_known_attitude_from_identity);
    RUN_TEST(test_tracks_level_360_rotation);
    RUN_TEST(test_mag_dropout_falls_back_without_corrupting_state);
    return UNITY_END();
}
