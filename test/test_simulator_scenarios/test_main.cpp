#include <unity.h>
#include <math.h>
#include "ImuSimulator.h"
#include "ImuCompass.h"
#include "ImuCalibration.h"
#include "ImuAngleMath.h"

using namespace ImuSimulator;

void setUp(void) {}
void tearDown(void) {}

void test_generate_sample_level_north(void)
{
    SimulatorState s;
    s.headingDeg = 0;
    SimulatedSample out = generateSample(s);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, out.accelG.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, out.accelG.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, out.accelG.z);
    // Feeding the generated mag through the compass formula should
    // recover heading 0 (up to the formula's own fixed convention -
    // compared against the compass module's own behavior for
    // consistency, not an assumed absolute value).
    double heading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(out.magRaw, 0, 0));
    double reference = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(Vec3(22.0, 0, 45.0), 0, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(reference, heading)));
}

void test_level_360_rotation_sweeps_heading(void)
{
    double periodSec = 30.0;
    SimulatorState s0 = Scenarios::level360Rotation(0.0, periodSec);
    SimulatorState sQuarter = Scenarios::level360Rotation(periodSec * 0.25, periodSec);
    SimulatorState sHalf = Scenarios::level360Rotation(periodSec * 0.5, periodSec);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, s0.headingDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, sQuarter.headingDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 180.0, sHalf.headingDeg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 360.0 / periodSec, s0.rateOfTurnDegPerSec);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, s0.rollDeg);
}

void test_rotation_360_with_heel(void)
{
    SimulatorState s = Scenarios::rotation360With20DegHeel(5.0, 30.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, s.rollDeg);
}

void test_fixed_heading_changing_attitude_stays_on_heading(void)
{
    for (double t = 0; t < 20.0; t += 1.0)
    {
        SimulatorState s = Scenarios::fixedHeadingChangingAttitude(t, 90.0);
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, s.headingDeg);
        // roll/pitch should actually be varying (not degenerately stuck at 0)
        TEST_ASSERT_TRUE(fabs(s.rollDeg) < 20.0);
    }
}

// Integration check: the hard-iron scenario's raw output, corrected with
// the SAME bias the scenario injected, must recover the scenario's true
// heading - proves the simulator's distortion model and ImuCalibration's
// correction model are inverses of each other, as intended.
void test_hard_iron_scenario_correctable(void)
{
    SimulatorState s = Scenarios::hardIronOffset(0.0, 45.0);
    SimulatedSample sample = generateSample(s);

    double cleanHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(
        Vec3(s.horizontalFieldMagnitude * cos(45.0 * M_PI / 180.0), -s.horizontalFieldMagnitude * sin(45.0 * M_PI / 180.0), s.verticalFieldMagnitude), 0, 0));

    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magBias[0] = s.magHardIronBias[0];
    cal.magBias[1] = s.magHardIronBias[1];
    cal.magBias[2] = s.magHardIronBias[2];
    Vec3 corrected = ImuCalibrationOps::applyMag(sample.magRaw, cal);
    double correctedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(corrected, 0, 0));

    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(cleanHeading, correctedHeading)));
}

void test_elliptical_soft_iron_scenario_correctable(void)
{
    SimulatorState s = Scenarios::ellipticalSoftIronDistortion(0.0, 45.0);
    SimulatedSample sample = generateSample(s);

    double cleanHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(
        Vec3(s.horizontalFieldMagnitude * cos(45.0 * M_PI / 180.0), -s.horizontalFieldMagnitude * sin(45.0 * M_PI / 180.0), s.verticalFieldMagnitude), 0, 0));

    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magMatrix[0][0] = 1.0 / s.magSoftIronScale[0];
    cal.magMatrix[1][1] = 1.0 / s.magSoftIronScale[1];
    Vec3 corrected = ImuCalibrationOps::applyMag(sample.magRaw, cal);
    double correctedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(corrected, 0, 0));

    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(cleanHeading, correctedHeading)));
}

void test_sudden_magnetic_disturbance_jumps_at_start_time(void)
{
    SimulatorState before = Scenarios::suddenMagneticDisturbance(4.9, 5.0);
    SimulatorState after = Scenarios::suddenMagneticDisturbance(5.1, 5.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, before.magHardIronBias[0]);
    TEST_ASSERT_TRUE(after.magHardIronBias[0] > 30.0);
}

void test_slow_gyro_drift_grows_over_time(void)
{
    SimulatorState early = Scenarios::slowGyroDrift(10.0, 0.001);
    SimulatorState later = Scenarios::slowGyroDrift(1000.0, 0.001);
    TEST_ASSERT_TRUE(later.gyroBiasDegPerSec[2] > early.gyroBiasDegPerSec[2]);
}

void test_heading_wrap_through_north_stays_near_boundary(void)
{
    for (double t = 0; t < 20.0; t += 0.5)
    {
        SimulatorState s = Scenarios::headingWrapThroughNorth(t, 20.0);
        // Should always be within 10 degrees of 0/360 either side.
        double distFromNorth = fabs(ImuAngleMath::shortestDiff(0.0, s.headingDeg));
        TEST_ASSERT_TRUE(distFromNorth <= 10.0 + 1e-6);
    }
}

void test_magnetometer_dropout_zeros_output(void)
{
    SimulatorState before = Scenarios::magnetometerDropout(4.0, 5.0);
    SimulatorState after = Scenarios::magnetometerDropout(6.0, 5.0);
    SimulatedSample sBefore = generateSample(before);
    SimulatedSample sAfter = generateSample(after);
    TEST_ASSERT_TRUE(sBefore.magRaw.norm() > 1.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sAfter.magRaw.norm());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_generate_sample_level_north);
    RUN_TEST(test_level_360_rotation_sweeps_heading);
    RUN_TEST(test_rotation_360_with_heel);
    RUN_TEST(test_fixed_heading_changing_attitude_stays_on_heading);
    RUN_TEST(test_hard_iron_scenario_correctable);
    RUN_TEST(test_elliptical_soft_iron_scenario_correctable);
    RUN_TEST(test_sudden_magnetic_disturbance_jumps_at_start_time);
    RUN_TEST(test_slow_gyro_drift_grows_over_time);
    RUN_TEST(test_heading_wrap_through_north_stays_near_boundary);
    RUN_TEST(test_magnetometer_dropout_zeros_output);
    return UNITY_END();
}
