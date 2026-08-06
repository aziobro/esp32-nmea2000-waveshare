#include <unity.h>
#include <math.h>
#include "ImuCycleProcessor.h"
#include "ImuQuaternion.h"

void setUp(void) {}
void tearDown(void) {}

static ImuCycleInput levelInput()
{
    ImuCycleInput in;
    in.accelBoat = Vec3(0, 0, 1.0); // level, 1g down
    in.gyroBoat = Vec3(0, 0, 0);
    in.magBoat = Vec3(50, 0, 0); // 50 uT, within MagMonitorConfig's default plausible range
    in.dtSec = 0.1;
    in.nowMs = 100000;
    in.taskStartMs = 0; // well past both DMP and fusion settle times by default
    in.headingMode = HeadingSourceMode::Auto;
    in.transitionMs = 0; // no blending - snap immediately, simpler to assert on
    return in;
}

void test_level_no_dmp_uses_accel_tilt_and_compass(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
    in.taskStartMs = in.nowMs; // just started - fusion's settle-time gate hasn't cleared yet
    ImuCycleOutput out = proc.process(in);

    TEST_ASSERT_TRUE(out.attitudeValid);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, out.rollDeg);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0, out.pitchDeg);
    TEST_ASSERT_TRUE(out.headingValid);
    // DMP is off entirely and fusion needs several seconds to be trusted -
    // compass should be the only usable candidate on this very first cycle.
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::SoftwareCompass);
}

void test_attitude_invalid_when_dmp_required_but_no_sample_yet(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = true;
    in.haveDmpSample = false; // DMP enabled but nothing read yet
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_FALSE(out.attitudeValid);
}

void test_dmp_becomes_active_source_after_consecutive_valid_samples(void)
{
    ImuCycleProcessor proc;
    Quaternion level = ImuQuaternion::fromEuler(0, 0, 0); // level, heading 0 (north)

    ImuCycleOutput out;
    unsigned long t = 100000;
    for (int i = 0; i < 6; i++)
    {
        ImuCycleInput in = levelInput();
        in.headingMode = HeadingSourceMode::Dmp; // pin to DMP explicitly - only valid if DMP validates
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = true;
        in.dmpQuat = level;
        in.dmpAgeMs = 10;
        in.nowMs = t;
        t += 100;
        out = proc.process(in);
    }
    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::Dmp);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 0.0, out.headingDeg);
}

void test_heading_offset_and_invert_applied(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
    ImuCycleOutput baseline = proc.process(in);
    TEST_ASSERT_TRUE(baseline.headingValid);

    ImuCycleProcessor proc2;
    ImuCycleInput in2 = levelInput();
    in2.dmpOk = false;
    in2.hdgOffsetDeg = 10.0;
    ImuCycleOutput withOffset = proc2.process(in2);
    TEST_ASSERT_TRUE(withOffset.headingValid);

    double diff = withOffset.headingDeg - baseline.headingDeg;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 10.0, diff);

    // Invert flips the sign of the pre-offset candidate heading - use a
    // mag vector with a nonzero Y component so north (0 deg) isn't a
    // degenerate fixed point of the sign flip.
    ImuCycleProcessor proc3;
    ImuCycleInput in3 = levelInput();
    in3.dmpOk = false;
    in3.magBoat = Vec3(50, 50, 0);
    ImuCycleOutput notInverted = proc3.process(in3);

    ImuCycleProcessor proc4;
    ImuCycleInput in4 = levelInput();
    in4.dmpOk = false;
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
    dev.addEntry(0.0, 5.0); // at measured heading 0, true heading is 5

    ImuCycleProcessor procOff;
    ImuCycleInput inOff = levelInput();
    inOff.dmpOk = false;
    inOff.deviationEnabled = false;
    inOff.deviationTable = dev;
    ImuCycleOutput outOff = procOff.process(inOff);

    ImuCycleProcessor procOn;
    ImuCycleInput inOn = levelInput();
    inOn.dmpOk = false;
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
    in.dmpOk = false;
    in.taskStartMs = 100000;
    in.nowMs = 100500; // only 500ms since start - well under the 3s settle time
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_FALSE(out.fusionCandidateValid);
}

void test_auto_falls_back_to_compass_when_fusion_disagrees(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
    in.magBoat = Vec3(0, 50, 0); // compass is far from fusion's identity-start yaw
    ImuCycleOutput out = proc.process(in);

    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_FALSE(out.fusionCandidateValid);
    TEST_ASSERT_TRUE((out.rejectionFlags & HR_FUSION_COMPASS_DISAGREE) != 0);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::SoftwareCompass);
}

void test_invalid_mag_rejects_all_mag_heading_sources(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
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
    in.dmpOk = false;
    in.rotFiltAlpha = 0.1; // heavy smoothing
    in.gyroBoat = Vec3(0, 0, 0);
    ImuCycleOutput first = proc.process(in); // establishes the filter state at 0

    in.gyroBoat = Vec3(0, 0, 100.0); // sudden 100 deg/s step
    in.nowMs += 100;
    ImuCycleOutput second = proc.process(in);

    // Heavily smoothed - should move toward 100 but not reach it in one step.
    TEST_ASSERT_TRUE(second.rotDegPerSec > 1.0);
    TEST_ASSERT_TRUE(second.rotDegPerSec < 50.0);
}

void test_mag_disturbance_state_reported(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
    in.magBoat = Vec3(5000, 0, 0); // wildly implausible magnitude
    ImuCycleOutput out = proc.process(in);
    TEST_ASSERT_TRUE(out.magMagnitude > 1000.0);
}

void test_holdover_engages_when_pinned_dmp_source_is_lost(void)
{
    ImuCycleProcessor proc;
    Quaternion level = ImuQuaternion::fromEuler(0, 0, 0);

    ImuCycleOutput out;
    unsigned long t = 100000;
    for (int i = 0; i < 5; i++)
    {
        ImuCycleInput in = levelInput();
        in.headingMode = HeadingSourceMode::Dmp; // pinned - no compass/fusion fallback
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = true;
        in.dmpQuat = level;
        in.dmpAgeMs = 10;
        in.nowMs = t;
        t += 100;
        out = proc.process(in);
    }
    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_FALSE(out.headingHoldover);

    // Now lose DMP entirely - stale samples (age well beyond
    // DmpValidationConfig's default maxSampleAgeMs of 500ms).
    for (int i = 0; i < 3; i++)
    {
        ImuCycleInput in = levelInput();
        in.headingMode = HeadingSourceMode::Dmp;
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = false;
        in.dmpQuat = level;
        in.dmpAgeMs = 5000;
        in.nowMs = t;
        t += 100;
        out = proc.process(in);
    }
    // Confirms several Phase 8 requirements at once: a lost source
    // doesn't immediately report invalid (holdover bridges it), the
    // holdover is clearly identified (headingHoldover=true), and the raw
    // selector's own reporting (headingSource) correctly shows no source
    // is actually active right now.
    TEST_ASSERT_TRUE(out.headingValid);
    TEST_ASSERT_TRUE(out.headingHoldover);
    TEST_ASSERT_TRUE(out.headingSource == HeadingSource::None);
}

void test_roll_pitch_continue_independently_of_heading_holdover(void)
{
    // Roll/pitch validity must never depend on whether the heading is
    // currently tracking, in holdover, or lost - it's DMP-euler-derived
    // (or accel-tilt) and computed unconditionally every cycle.
    ImuCycleProcessor proc;
    Quaternion level = ImuQuaternion::fromEuler(0, 0, 0);

    unsigned long t = 100000;
    for (int i = 0; i < 5; i++)
    {
        ImuCycleInput in = levelInput();
        in.headingMode = HeadingSourceMode::Dmp;
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = true;
        in.dmpQuat = level;
        in.dmpAgeMs = 10;
        in.nowMs = t;
        t += 100;
        proc.process(in);
    }

    // Lose heading (DMP stale) for several cycles - attitude must stay valid throughout.
    for (int i = 0; i < 5; i++)
    {
        ImuCycleInput in = levelInput();
        in.headingMode = HeadingSourceMode::Dmp;
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = false;
        in.dmpQuat = level;
        in.dmpAgeMs = 5000;
        in.nowMs = t;
        t += 100;
        ImuCycleOutput out = proc.process(in);
        TEST_ASSERT_TRUE_MESSAGE(out.attitudeValid, "roll/pitch must stay valid even while heading is lost/holdover");
    }
}

void test_fusion_duration_reported(void)
{
    ImuCycleProcessor proc;
    ImuCycleInput in = levelInput();
    in.dmpOk = false;
    ImuCycleOutput out = proc.process(in);
    // Sanity range only - a real duration varies by machine, this just
    // confirms the field is actually being measured and not left at a
    // stale/garbage value.
    TEST_ASSERT_TRUE(out.fusionDurationUs >= 0.0);
    TEST_ASSERT_TRUE(out.fusionDurationUs < 100000.0);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_no_dmp_uses_accel_tilt_and_compass);
    RUN_TEST(test_attitude_invalid_when_dmp_required_but_no_sample_yet);
    RUN_TEST(test_dmp_becomes_active_source_after_consecutive_valid_samples);
    RUN_TEST(test_heading_offset_and_invert_applied);
    RUN_TEST(test_deviation_table_applied_when_enabled);
    RUN_TEST(test_fusion_invalid_before_settle_time);
    RUN_TEST(test_auto_falls_back_to_compass_when_fusion_disagrees);
    RUN_TEST(test_invalid_mag_rejects_all_mag_heading_sources);
    RUN_TEST(test_rot_filter_smooths_across_cycles);
    RUN_TEST(test_mag_disturbance_state_reported);
    RUN_TEST(test_holdover_engages_when_pinned_dmp_source_is_lost);
    RUN_TEST(test_roll_pitch_continue_independently_of_heading_holdover);
    RUN_TEST(test_fusion_duration_reported);
    return UNITY_END();
}
