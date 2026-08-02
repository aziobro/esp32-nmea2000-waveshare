#include <unity.h>
#include "ImuHeadingHoldover.h"

void setUp(void) {}
void tearDown(void) {}

void test_starts_lost_when_never_had_a_source(void)
{
    ImuHeadingHoldover h;
    auto r = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 1000);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Lost);
}

void test_tracks_directly_when_source_valid(void)
{
    ImuHeadingHoldover h;
    auto r = h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 1000);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, r.headingDeg);
    TEST_ASSERT_TRUE(r.quality == HeadingQuality::Good);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Tracking);
    TEST_ASSERT_FALSE(r.gyroOnly);
}

void test_enters_holdover_when_source_lost_after_tracking(void)
{
    ImuHeadingHoldover h;
    h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 1000);
    auto r = h.update(false, 0, HeadingQuality::Invalid, 5.0, 0.1, 1100);
    TEST_ASSERT_TRUE(r.valid); // holdover - not immediately invalid
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Holdover);
    TEST_ASSERT_TRUE(r.gyroOnly);
    // integrated forward by rot*dt = 5*0.1 = 0.5 deg
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.5, r.headingDeg);
}

void test_quality_downgrades_to_poor_during_holdover(void)
{
    ImuHeadingHoldover h;
    h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 1000);
    auto r = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 1100);
    TEST_ASSERT_TRUE(r.quality == HeadingQuality::Poor);
}

void test_holdover_integrates_rate_of_turn_across_multiple_cycles(void)
{
    ImuHeadingHoldover h;
    h.update(true, 350.0, HeadingQuality::Good, 0, 0.1, 1000);
    ImuHeadingHoldover::Result r;
    unsigned long t = 1000;
    for (int i = 0; i < 10; i++)
    {
        t += 100;
        r = h.update(false, 0, HeadingQuality::Invalid, 20.0, 0.1, t); // 20 deg/s turn to starboard
    }
    // 350 + 10*20*0.1 = 350 + 20 = 370 -> wraps to 10
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, 10.0, r.headingDeg);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Holdover);
}

void test_holdover_expires_after_max_duration_and_becomes_lost(void)
{
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.maxHoldoverMs = 500;
    h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 1000, cfg);
    // Holdover's own clock starts at the FIRST loss cycle (1400), not at
    // the last tracked sample (1000) - elapsed below is relative to that.
    auto during = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 1400, cfg);
    TEST_ASSERT_TRUE(during.valid); // 0ms elapsed < 500ms limit
    auto stillDuring = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 1800, cfg);
    TEST_ASSERT_TRUE(stillDuring.valid); // 400ms elapsed < 500ms limit
    auto after = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 2000, cfg);
    TEST_ASSERT_FALSE(after.valid); // 600ms elapsed > 500ms limit
    TEST_ASSERT_TRUE(after.state == HeadingHoldoverState::Lost);
}

void test_stale_heading_not_held_indefinitely(void)
{
    // Same as the expiry test but framed as the spec's own wording:
    // holdover must eventually give up, not hold a stale value forever.
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.maxHoldoverMs = 1000;
    h.update(true, 45.0, HeadingQuality::Good, 0, 0.1, 0, cfg);
    ImuHeadingHoldover::Result r;
    unsigned long t = 0;
    for (int i = 0; i < 50; i++) // 50 * 100ms = 5s of continued loss
    {
        t += 100;
        r = h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, t, cfg);
    }
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Lost);
}

void test_recovery_requires_consecutive_valid_samples(void)
{
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.minConsecutiveSamplesToRecover = 3;
    h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 1000, cfg);
    h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 1100, cfg); // -> Holdover

    // First valid sample back - not enough yet, should still be holdover-ish.
    auto r1 = h.update(true, 200.0, HeadingQuality::Good, 0, 0.1, 1200, cfg);
    TEST_ASSERT_FALSE(r1.state == HeadingHoldoverState::Tracking);

    auto r2 = h.update(true, 200.0, HeadingQuality::Good, 0, 0.1, 1300, cfg);
    TEST_ASSERT_FALSE(r2.state == HeadingHoldoverState::Tracking);

    // Third consecutive valid sample - now recovering (blend in progress
    // or fully tracking, depending on recoveryBlendMs, but not "invalid").
    auto r3 = h.update(true, 200.0, HeadingQuality::Good, 0, 0.1, 1400, cfg);
    TEST_ASSERT_TRUE(r3.valid);
}

void test_recovery_uses_circular_interpolation_not_instant_jump(void)
{
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.minConsecutiveSamplesToRecover = 1;
    cfg.recoveryBlendMs = 1000;
    h.update(true, 10.0, HeadingQuality::Good, 0, 0.1, 0, cfg);
    h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 100, cfg); // Holdover, held near 10

    // 10 -> 100 deg: unambiguously shortest going forward (90 deg) rather
    // than backward through north (270 deg), so the mid-blend value's
    // expected direction isn't ambiguous. The recovery blend's own clock
    // starts on the first call where the consecutive-samples threshold is
    // met (600 here, elapsed=0) - a second call partway through that same
    // blend window is what shows gradual movement, not the first one.
    h.update(true, 100.0, HeadingQuality::Good, 0, 0.1, 600, cfg);
    auto mid = h.update(true, 100.0, HeadingQuality::Good, 0, 0.1, 1100, cfg); // 500ms into the 1000ms blend
    TEST_ASSERT_TRUE(mid.valid);
    // Should be somewhere between 10 and 100, not already at 100 and not
    // still at 10.
    TEST_ASSERT_TRUE(mid.headingDeg > 10.0);
    TEST_ASSERT_TRUE(mid.headingDeg < 100.0);
}

void test_north_crossing_during_recovery_does_not_jump_360(void)
{
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.minConsecutiveSamplesToRecover = 1;
    cfg.recoveryBlendMs = 1000;
    h.update(true, 350.0, HeadingQuality::Good, 0, 0.1, 0, cfg);
    h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 100, cfg);

    // Recovered source is just past north (10 deg) - shortest path is
    // +20 through 360/0, not backward through 180.
    auto mid = h.update(true, 10.0, HeadingQuality::Good, 0, 0.1, 600, cfg);
    TEST_ASSERT_TRUE(mid.valid);
    // Should be near the 350-360-10 arc, not anywhere near 180.
    bool nearWrapArc = (mid.headingDeg > 340.0) || (mid.headingDeg < 20.0);
    TEST_ASSERT_TRUE(nearWrapArc);
}

void test_magnetic_disturbance_loss_still_allows_holdover(void)
{
    // Holdover doesn't know or care WHY sourceValid became false - a
    // magnetic-disturbance-caused loss must trigger holdover exactly
    // like any other loss, not be specially rejected.
    ImuHeadingHoldover h;
    h.update(true, 120.0, HeadingQuality::Good, 0, 0.1, 1000);
    // Simulate what the pipeline would report when magnetic disturbance
    // invalidates every candidate: sourceValid=false, same as any cause.
    auto r = h.update(false, 0, HeadingQuality::Invalid, 3.0, 0.1, 1100);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.gyroOnly);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Holdover);
}

void test_lost_recovery_snaps_without_blend(void)
{
    ImuHeadingHoldover h;
    HeadingHoldoverConfig cfg;
    cfg.maxHoldoverMs = 100;
    cfg.minConsecutiveSamplesToRecover = 1;
    h.update(true, 90.0, HeadingQuality::Good, 0, 0.1, 0, cfg);
    h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 50, cfg);  // Holdover
    h.update(false, 0, HeadingQuality::Invalid, 0, 0.1, 300, cfg); // now Lost (250ms > 100ms limit)

    auto r = h.update(true, 200.0, HeadingQuality::Good, 0, 0.1, 400, cfg);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.state == HeadingHoldoverState::Tracking);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 200.0, r.headingDeg); // direct snap, no blend
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_lost_when_never_had_a_source);
    RUN_TEST(test_tracks_directly_when_source_valid);
    RUN_TEST(test_enters_holdover_when_source_lost_after_tracking);
    RUN_TEST(test_quality_downgrades_to_poor_during_holdover);
    RUN_TEST(test_holdover_integrates_rate_of_turn_across_multiple_cycles);
    RUN_TEST(test_holdover_expires_after_max_duration_and_becomes_lost);
    RUN_TEST(test_stale_heading_not_held_indefinitely);
    RUN_TEST(test_recovery_requires_consecutive_valid_samples);
    RUN_TEST(test_recovery_uses_circular_interpolation_not_instant_jump);
    RUN_TEST(test_north_crossing_during_recovery_does_not_jump_360);
    RUN_TEST(test_magnetic_disturbance_loss_still_allows_holdover);
    RUN_TEST(test_lost_recovery_snaps_without_blend);
    return UNITY_END();
}
