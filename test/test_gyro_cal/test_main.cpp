#include <unity.h>
#include <math.h>
#include "ImuGyroCal.h"

void setUp(void) {}
void tearDown(void) {}

void test_starts_idle_then_collecting(void)
{
    GyroCalEngine e;
    TEST_ASSERT_EQUAL_INT((int)GyroCalState::Idle, (int)e.state());
    e.start();
    TEST_ASSERT_EQUAL_INT((int)GyroCalState::Collecting, (int)e.state());
    TEST_ASSERT_EQUAL_INT(0, e.sampleCount());
}

void test_rejects_high_motion_sample(void)
{
    GyroCalEngine e;
    e.start();
    GyroCalConfig cfg;
    cfg.maxGyroMagDegPerSec = 2.0;
    bool accepted = e.addSample(Vec3(5.0, 0, 0), 1.0, cfg); // 5 deg/s, over the 2.0 gate
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_EQUAL_INT(0, e.sampleCount());
}

void test_rejects_non_level_accel(void)
{
    GyroCalEngine e;
    e.start();
    GyroCalConfig cfg;
    cfg.accelMagToleranceG = 0.05;
    bool accepted = e.addSample(Vec3(0, 0, 0), 1.3, cfg); // accelerating/tilted, not stationary
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_EQUAL_INT(0, e.sampleCount());
}

void test_accepts_good_sample(void)
{
    GyroCalEngine e;
    e.start();
    bool accepted = e.addSample(Vec3(0.1, -0.05, 0.2), 1.0);
    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_EQUAL_INT(1, e.sampleCount());
}

void test_converges_to_known_constant_bias(void)
{
    GyroCalEngine e;
    e.start();
    GyroCalConfig cfg;
    cfg.requiredSamples = 100;
    cfg.maxStdDevDegPerSec = 1.0;
    for (int i = 0; i < 100; i++)
        e.addSample(Vec3(0.5, -0.3, 0.8), 1.0, cfg); // constant, zero-noise bias
    TEST_ASSERT_EQUAL_INT((int)GyroCalState::Done, (int)e.state());
    Vec3 bias = e.resultBiasDegPerSec();
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5, bias.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.3, bias.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.8, bias.z);
    Vec3 stddev = e.resultStdDevDegPerSec();
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, stddev.x);
}

void test_fails_on_unstable_noisy_samples(void)
{
    GyroCalEngine e;
    e.start();
    GyroCalConfig cfg;
    cfg.requiredSamples = 20;
    cfg.maxGyroMagDegPerSec = 100.0; // allow the noisy samples through the motion gate
    cfg.maxStdDevDegPerSec = 0.5;
    for (int i = 0; i < 20; i++)
    {
        double noisy = (i % 2 == 0) ? 5.0 : -5.0; // large alternating swing, high stddev
        e.addSample(Vec3(noisy, 0, 0), 1.0, cfg);
    }
    TEST_ASSERT_EQUAL_INT((int)GyroCalState::Failed, (int)e.state());
}

void test_cancel_resets_to_idle(void)
{
    GyroCalEngine e;
    e.start();
    e.addSample(Vec3(0.1, 0, 0), 1.0);
    e.cancel();
    TEST_ASSERT_EQUAL_INT((int)GyroCalState::Idle, (int)e.state());
    TEST_ASSERT_EQUAL_INT(0, e.sampleCount());
}

void test_low_pass_basic(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, ImuRateOfTurn::lowPass(10.0, 20.0, 0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, ImuRateOfTurn::lowPass(10.0, 20.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 15.0, ImuRateOfTurn::lowPass(10.0, 20.0, 0.5));
}

void test_derived_from_heading_matches_known_rate(void)
{
    // Heading went from 100 to 105 degrees (unwrapped) over 0.5s -> 10 deg/s.
    double rot = ImuRateOfTurn::derivedFromHeadingDegPerSec(105.0, 100.0, 0.5);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, rot);
}

void test_disagreement_detection(void)
{
    TEST_ASSERT_FALSE(ImuRateOfTurn::disagreesWithHeadingDerivative(10.0, 10.5, 2.0));
    TEST_ASSERT_TRUE(ImuRateOfTurn::disagreesWithHeadingDerivative(10.0, 25.0, 2.0));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_idle_then_collecting);
    RUN_TEST(test_rejects_high_motion_sample);
    RUN_TEST(test_rejects_non_level_accel);
    RUN_TEST(test_accepts_good_sample);
    RUN_TEST(test_converges_to_known_constant_bias);
    RUN_TEST(test_fails_on_unstable_noisy_samples);
    RUN_TEST(test_cancel_resets_to_idle);
    RUN_TEST(test_low_pass_basic);
    RUN_TEST(test_derived_from_heading_matches_known_rate);
    RUN_TEST(test_disagreement_detection);
    return UNITY_END();
}
