#include <unity.h>
#include <math.h>
#include "ImuHeadingFilter.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

void test_first_update_snaps_to_value(void)
{
    HeadingFilter f;
    double out = f.update(45.0, 0.1, 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 45.0, out);
    TEST_ASSERT_TRUE(f.isInitialized());
}

void test_disabled_filter_passes_through(void)
{
    HeadingFilter f;
    f.reset(0.0);
    HeadingFilterConfig cfg;
    cfg.enabled = false;
    double out = f.update(90.0, 0.1, 0.0, cfg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, out);
}

void test_smooths_small_steady_changes(void)
{
    HeadingFilter f;
    f.reset(0.0);
    HeadingFilterConfig cfg;
    cfg.timeConstantSec = 1.0;
    cfg.maxJumpDegPerSample = 30.0;
    // Small step, short dt relative to tau - output should move toward
    // the target but not reach it instantly.
    double out = f.update(10.0, 0.05, 0.0, cfg);
    TEST_ASSERT_TRUE(out > 0.0 && out < 10.0);
}

void test_converges_over_many_steady_updates(void)
{
    HeadingFilter f;
    f.reset(0.0);
    HeadingFilterConfig cfg;
    cfg.timeConstantSec = 0.5;
    cfg.maxJumpDegPerSample = 30.0;
    double out = 0;
    for (int i = 0; i < 200; i++)
        out = f.update(20.0, 0.02, 0.0, cfg);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 20.0, out);
}

void test_large_jump_snaps_instead_of_smoothing(void)
{
    HeadingFilter f;
    f.reset(0.0);
    HeadingFilterConfig cfg;
    cfg.maxJumpDegPerSample = 30.0;
    double out = f.update(170.0, 0.02, 0.0, cfg); // way over the jump threshold
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 170.0, out);
}

void test_wrap_handled_correctly(void)
{
    HeadingFilter f;
    f.reset(355.0);
    HeadingFilterConfig cfg;
    cfg.timeConstantSec = 0.1;
    cfg.maxJumpDegPerSample = 30.0;
    double out = 0;
    for (int i = 0; i < 50; i++)
        out = f.update(5.0, 0.02, 0.0, cfg); // 355 -> 5 is a 10-degree step across the wrap
    // Should converge near 5 (via the SHORT path through 0), not regress
    // toward 180 (which plain arithmetic averaging would do).
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 0.0, fabs(ImuAngleMath::shortestDiff(5.0, out)));
}

void test_fast_time_constant_used_when_turning(void)
{
    HeadingFilterConfig cfg;
    cfg.timeConstantSec = 5.0;      // very slow normally
    cfg.fastTimeConstantSec = 0.01; // very fast when turning
    cfg.rotThresholdForFastDegPerSec = 5.0;
    cfg.maxJumpDegPerSample = 30.0;

    HeadingFilter slow;
    slow.reset(0.0);
    double outSlow = slow.update(20.0, 0.05, 0.0, cfg); // not turning

    HeadingFilter fast;
    fast.reset(0.0);
    double outFast = fast.update(20.0, 0.05, 10.0, cfg); // turning fast

    // The "turning" case should track much closer to the target than the
    // steady case, given the same input step and dt.
    TEST_ASSERT_TRUE(fabs(outFast - 20.0) < fabs(outSlow - 20.0));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_update_snaps_to_value);
    RUN_TEST(test_disabled_filter_passes_through);
    RUN_TEST(test_smooths_small_steady_changes);
    RUN_TEST(test_converges_over_many_steady_updates);
    RUN_TEST(test_large_jump_snaps_instead_of_smoothing);
    RUN_TEST(test_wrap_handled_correctly);
    RUN_TEST(test_fast_time_constant_used_when_turning);
    return UNITY_END();
}
