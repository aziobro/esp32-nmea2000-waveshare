#include <unity.h>
#include <math.h>
#include "ImuCycleProcessor.h"

void setUp(void) {}
void tearDown(void) {}

static ImuCycleInput levelInput()
{
    ImuCycleInput in;
    in.accelBoat = Vec3(0, 0, 1.0);
    in.gyroBoat = Vec3(0, 0, 0);
    in.magBoat = Vec3(50, 0, 0);
    in.dtSec = 0.1;
    in.nowMs = 100000;
    in.taskStartMs = 0;
    in.headingMode = HeadingSourceMode::Auto;
    in.transitionMs = 0;
    return in;
}

void test_level_uses_accel_tilt_and_compass_before_fusion_settles(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.taskStartMs = in.nowMs;
    ImuCycleOutput out = proc.process(in);

    TEST_ASSERT_TRUE(out.attitudeValid);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, out.rollDeg);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, out.pitchDeg);
    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::SoftwareCompass);
}

void test_heading_offset_and_invert_applied(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    ImuCycleOutput baseline = proc.process(in);
    TEST_ASSERT_TRUE(baseline.headingValid);

    ImuCycleProcessor proc2;
    ImuCycleInput in2 = levelInput();
    in2.hdgOffsetDeg = 10.0;
    ImuCycleOutput withOffset = proc2.process(in2);
    TEST_ASSERT_TRUE(withOffset.headingValid);

    double diff = withOffset.headingDeg - baseline.headingDeg;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 10.0, diff);

    ImuCycleProcessor proc3;
    ImuCycleInput in3 = levelInput();
    in3.magBoat = Vec3(50, 50, 0);
    ImuCycleOutput notInverted = proc3.process(in3);

    ImuCycleProcessor proc4;
    ImuCycleInput in4 = levelInput();
    in4.magBoat = Vec3(50, 50, 0);
    in4.hdgInvert = true;
    ImuCycleOutput inverted = proc4.process(in4);

    TEST_ASSERT_TRUE(notInverted.headingValid && inverted.headingValid);
    double expected = -notInverted.headingDeg;
    while (expected < 0) expected += 360;
    while (expected >= 360) expected -= 360;
    TEST_ASSERT_DOUBLE_WITHIN(0.5, expected, inverted.headingDeg);
}

void test_deviation_table_applied_when_enabled(void)
{
    DeviationTable dev;
    dev.addEntry(0.0, 5.0);

    ImuCycleProcessor procOff;
    ImuCycleInput inOff = levelInput();
    inOff.deviationEnabled = false;
    inOff.deviationTable = dev;
    ImuCycleOutput outOff = procOff.process(inOff);

    ImuCycleProcessor procOn;
    ImuCycleInput inOn = levelInput();
    inOn.deviationEnabled = true;
    inOn.deviationTable = dev;
    ImuCycleOutput outOn = procOn.process(inOn);

    TEST_ASSERT_TRUE(outOff.headingValid && outOn.headingValid);
    TEST_ASSERT_TRUE(fabs(outOn.headingDeg - outOff.headingDeg) > 0.1);
}

void test_fusion_invalid_before_settle_time(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.taskStartMs = 100000;
    in.nowMs = 100500;
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_FALSE(out.fusionCandidateValid);
}

void test_auto_falls_back_to_compass_when_fusion_disagrees(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.magBoat = Vec3(0, 50, 0);
    ImuCycleOutput out = proc.process(in);

    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_FALSE(out.fusionCandidateValid);
    TEST_ASSERT_TRUE((out.rejectionFlags & HR_FUSION_COMPASS_DISAGREE) != 0);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::SoftwareCompass);
}

void test_auto_uses_fusion_after_static_convergence_with_compass_sign_convention(void)
{
    ImuCycleProcessor proc;
    ImuCycleOutput out;
    ImuCycleInput in = levelInput();
    in.magBoat = Vec3(-20, 16, 38);
    in.dtSec = 0.02;
    in.nowMs = 100000;

    for (int i = 0; i < 5000; i++)
    {
        in.nowMs += 20;
        out = proc.process(in);
    }

    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_TRUE(out.fusionCandidateValid);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::SoftwareFusion);
    TEST_ASSERT_DOUBLE_WITHIN(5.0, 0.0, fabs(ImuAngleMath::shortestDiff(out.rawCompassHeadingDeg, out.rawFusionHeadingDeg)));
}

void test_invalid_mag_rejects_all_mag_heading_sources(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.magBoat = Vec3(0, 0, 0);
    in.magValid = false;

    ImuCycleOutput out = proc.process(in);

    TEST_ASSERT_FALSE(out.headingValid);
    TEST_ASSERT_FALSE(out.fusionCandidateValid);
    TEST_ASSERT_TRUE((out.rejectionFlags & HR_MAG_INVALID) != 0);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::None);
}

void test_rot_filter_smooths_across_cycles(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.rotFiltAlpha = 0.1;
    in.gyroBoat = Vec3(0, 0, 0);
    proc.process(in);

    in.gyroBoat = Vec3(0, 0, 100.0);
    in.nowMs += 100;
    ImuCycleOutput second = proc.process(in);

    TEST_ASSERT_TRUE(second.rotDegPerSec > 1.0);
    TEST_ASSERT_TRUE(second.rotDegPerSec < 50.0);
}

void test_mag_disturbance_state_reported(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.magBoat = Vec3(5000, 0, 0);
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_TRUE(out.magMagnitude > 1000.0);
}

void test_roll_pitch_continue_when_heading_is_lost(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.magValid = false;
    in.magBoat = Vec3(0, 0, 0);
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_TRUE(out.attitudeValid);
    TEST_ASSERT_FALSE(out.headingValid);
}

void test_fusion_duration_reported(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_TRUE(out.fusionDurationUs >= 0.0);
    TEST_ASSERT_TRUE(out.fusionDurationUs < 100000.0);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_uses_accel_tilt_and_compass_before_fusion_settles);
    RUN_TEST(test_heading_offset_and_invert_applied);
    RUN_TEST(test_deviation_table_applied_when_enabled);
    RUN_TEST(test_fusion_invalid_before_settle_time);
    RUN_TEST(test_auto_falls_back_to_compass_when_fusion_disagrees);
    RUN_TEST(test_auto_uses_fusion_after_static_convergence_with_compass_sign_convention);
    RUN_TEST(test_invalid_mag_rejects_all_mag_heading_sources);
    RUN_TEST(test_rot_filter_smooths_across_cycles);
    RUN_TEST(test_mag_disturbance_state_reported);
    RUN_TEST(test_roll_pitch_continue_when_heading_is_lost);
    RUN_TEST(test_fusion_duration_reported);
    return UNITY_END();
}
