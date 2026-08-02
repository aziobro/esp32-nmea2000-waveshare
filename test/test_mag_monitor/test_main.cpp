#include <unity.h>
#include "ImuMagMonitor.h"

void setUp(void) {}
void tearDown(void) {}

void test_tracks_magnitude_and_min_max(void)
{
    MagMonitor m;
    MagMonitorConfig cfg;
    m.update(Vec3(20, 0, 0), cfg);
    m.update(Vec3(22, 0, 0), cfg);
    m.update(Vec3(18, 0, 0), cfg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 18.0, m.instantaneousMagnitude());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 18.0, m.minMagnitudeSeen());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 22.0, m.maxMagnitudeSeen());
}

void test_stays_good_with_steady_field(void)
{
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.hysteresisSamples = 3;
    for (int i = 0; i < 10; i++)
        m.update(Vec3(22.0, 0, 0), cfg);
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Good, (int)m.state());
}

void test_flags_out_of_range_magnitude(void)
{
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.minMagnitude = 15.0;
    cfg.maxMagnitude = 80.0;
    cfg.hysteresisSamples = 3;
    for (int i = 0; i < 5; i++)
        m.update(Vec3(5.0, 0, 0), cfg); // well under minMagnitude
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Disturbed, (int)m.state());
}

void test_flags_sudden_change(void)
{
    // The sudden-change check compares each sample against the
    // immediately previous one (by design - it's meant to catch abrupt
    // transitions, not a level that's simply different from calibration
    // time, which is the separate reference-deviation check tested
    // below) - so a single jump that then holds steady only flags the
    // one transition sample, not a sustained disturbance. To actually
    // accumulate enough consecutive bad samples to flip state via
    // hysteresis, the field needs to keep jumping sample-to-sample, e.g.
    // something moving/vibrating near the sensor.
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.hysteresisSamples = 3;
    cfg.maxSuddenChangePercent = 20.0;
    for (int i = 0; i < 5; i++)
        m.update(Vec3(22.0, 0, 0), cfg);
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Good, (int)m.state());
    for (int i = 0; i < 5; i++)
        m.update(Vec3((i % 2 == 0) ? 42.0 : 10.0, 0, 0), cfg);
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Disturbed, (int)m.state());
}

void test_stabilizing_at_new_level_only_flags_transiently(void)
{
    // The companion case to the above: a one-time jump that then holds
    // steady should NOT stay flagged as disturbed via the sudden-change
    // check alone (with no reference magnitude configured) - it's a
    // legitimate transient, not sustained. This documents that
    // behavior deliberately, rather than leaving it as a surprise.
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.hysteresisSamples = 3;
    cfg.maxSuddenChangePercent = 20.0;
    for (int i = 0; i < 5; i++)
        m.update(Vec3(22.0, 0, 0), cfg);
    for (int i = 0; i < 5; i++)
        m.update(Vec3(42.0, 0, 0), cfg); // jumps once, then holds steady at the new level
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Good, (int)m.state());
}

void test_hysteresis_ignores_single_bad_sample(void)
{
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.hysteresisSamples = 5;
    for (int i = 0; i < 10; i++)
        m.update(Vec3(22.0, 0, 0), cfg);
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Good, (int)m.state());
    // One isolated bad sample, then back to good - shouldn't flip state
    // (needs 5 consecutive bad samples to flip).
    m.update(Vec3(1.0, 0, 0), cfg);
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Good, (int)m.state());
}

void test_reference_deviation_check(void)
{
    MagMonitor m;
    MagMonitorConfig cfg;
    cfg.referenceMagnitude = 22.0;
    cfg.maxDeviationFromReferencePercent = 10.0;
    cfg.hysteresisSamples = 3;
    for (int i = 0; i < 5; i++)
        m.update(Vec3(30.0, 0, 0), cfg); // >10% off the 22.0 reference
    TEST_ASSERT_EQUAL_INT((int)MagDisturbanceState::Disturbed, (int)m.state());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_tracks_magnitude_and_min_max);
    RUN_TEST(test_stays_good_with_steady_field);
    RUN_TEST(test_flags_out_of_range_magnitude);
    RUN_TEST(test_flags_sudden_change);
    RUN_TEST(test_stabilizing_at_new_level_only_flags_transiently);
    RUN_TEST(test_hysteresis_ignores_single_bad_sample);
    RUN_TEST(test_reference_deviation_check);
    return UNITY_END();
}
