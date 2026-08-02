#include <unity.h>
#include <math.h>
#include <string.h>
#include "ImuMagCal2D.h"

void setUp(void) {}
void tearDown(void) {}

static void feedCircle(ImuMagCal2D &cal, int n, double biasX, double biasY, double radius,
                        double scaleX = 1.0, double scaleY = 1.0, const MagCal2DConfig &cfg = MagCal2DConfig())
{
    for (int i = 0; i < n; i++)
    {
        double angle = (2.0 * M_PI * i) / n;
        double x = biasX + (radius / scaleX) * cos(angle);
        double y = biasY + (radius / scaleY) * sin(angle);
        cal.addSample(x, y, cfg);
    }
}

void test_idle_by_default(void)
{
    ImuMagCal2D cal;
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Idle);
    TEST_ASSERT_EQUAL_INT(0, cal.sampleCount());
}

void test_add_sample_ignored_when_not_collecting(void)
{
    ImuMagCal2D cal;
    bool accepted = cal.addSample(10.0, 0.0);
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_EQUAL_INT(0, cal.sampleCount());
}

void test_add_sample_rejects_implausible_magnitude(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    TEST_ASSERT_FALSE(cal.addSample(0.001, 0.001, cfg));  // far below minFieldMagnitudeUT
    TEST_ASSERT_FALSE(cal.addSample(9999.0, 9999.0, cfg)); // far above maxFieldMagnitudeUT
    TEST_ASSERT_EQUAL_INT(0, cal.sampleCount());
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Collecting); // rejection doesn't abort the session
}

void test_stop_fails_with_too_few_samples(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.minSamples = 200;
    feedCircle(cal, 10, 0, 0, 50.0, 1.0, 1.0, cfg);
    bool ok = cal.stop(cfg);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Failed);
    TEST_ASSERT_TRUE(strstr(cal.failureReason(), "samples") != nullptr);
}

void test_stop_fails_with_poor_coverage(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.minSamples = 50;
    cfg.sectorCount = 16;
    cfg.minCoverageFraction = 0.7;
    // 100 samples but all clustered in a narrow angular range - only a
    // couple of sectors ever touched.
    for (int i = 0; i < 100; i++)
    {
        double angle = (5.0 * M_PI / 180.0) * (i % 5); // 0..20 degrees only
        cal.addSample(50.0 * cos(angle), 50.0 * sin(angle), cfg);
    }
    bool ok = cal.stop(cfg);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Failed);
    TEST_ASSERT_TRUE(strstr(cal.failureReason(), "coverage") != nullptr);
}

void test_stop_succeeds_and_recovers_bias_for_clean_circle(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.minSamples = 200;
    cfg.sectorCount = 16;
    feedCircle(cal, 360, 12.0, -7.0, 45.0, 1.0, 1.0, cfg);
    bool ok = cal.stop(cfg);
    TEST_ASSERT_TRUE_MESSAGE(ok, cal.failureReason());
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Ready);
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 12.0, cal.resultBiasX());
    TEST_ASSERT_DOUBLE_WITHIN(0.05, -7.0, cal.resultBiasY());
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 1.0, cal.resultScaleX());
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 1.0, cal.resultScaleY());
    TEST_ASSERT_TRUE(cal.resultQuality() > 0.9);
}

void test_stop_recovers_axis_scale_for_elongated_ellipse(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.minSamples = 200;
    cfg.sectorCount = 16;
    cfg.maxResidualFraction = 0.5; // an axis-aligned ellipse isn't a circle - the 2D engine only corrects scale, so relax the circle-fit residual gate for this test
    feedCircle(cal, 360, 0.0, 0.0, 40.0, 1.5, 0.75, cfg); // x-radius=40/1.5, y-radius=40/0.75 - a real ellipse
    bool ok = cal.stop(cfg);
    TEST_ASSERT_TRUE_MESSAGE(ok, cal.failureReason());
    // scaleX/scaleY should reflect that the y-axis was stretched relative
    // to x (scaleY < 1 < scaleX, roughly matching the 1.5/0.75 input ratio).
    TEST_ASSERT_TRUE(cal.resultScaleX() > cal.resultScaleY());
    TEST_ASSERT_DOUBLE_WITHIN(0.15, 2.0, cal.resultScaleX() / cal.resultScaleY());
}

void test_stop_noop_when_not_collecting(void)
{
    ImuMagCal2D cal;
    bool ok = cal.stop();
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Idle);
}

void test_cancel_resets_to_idle(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    feedCircle(cal, 50, 0, 0, 40.0, 1.0, 1.0, cfg);
    cal.cancel();
    TEST_ASSERT_TRUE(cal.state() == MagCal2DState::Idle);
    TEST_ASSERT_EQUAL_INT(0, cal.sampleCount());
}

void test_coverage_fraction_tracks_visited_sectors(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.sectorCount = 4; // quadrants
    cal.addSample(10, 0, cfg);  // sector 0
    cal.addSample(0, 10, cfg);  // sector 1
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5, cal.coverageFraction(cfg));
    cal.addSample(-10, 0, cfg); // sector 2
    cal.addSample(0, -10, cfg); // sector 3
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, cal.coverageFraction(cfg));
}

void test_span_deg_full_circle(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.sectorCount = 8;
    feedCircle(cal, 200, 0, 0, 30.0, 1.0, 1.0, cfg);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 360.0, cal.spanDeg(cfg));
}

void test_span_deg_partial_arc(void)
{
    ImuMagCal2D cal;
    cal.start();
    MagCal2DConfig cfg;
    cfg.sectorCount = 8; // 45 deg/sector
    // Only cover sectors 0 and 1 (0-90 degrees).
    cal.addSample(10, 1, cfg);
    cal.addSample(1, 10, cfg);
    double span = cal.spanDeg(cfg);
    TEST_ASSERT_TRUE(span <= 90.0 + 1e-6);
    TEST_ASSERT_TRUE(span > 0.0);
}

void test_span_deg_zero_when_no_samples(void)
{
    ImuMagCal2D cal;
    cal.start();
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, cal.spanDeg());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_by_default);
    RUN_TEST(test_add_sample_ignored_when_not_collecting);
    RUN_TEST(test_add_sample_rejects_implausible_magnitude);
    RUN_TEST(test_stop_fails_with_too_few_samples);
    RUN_TEST(test_stop_fails_with_poor_coverage);
    RUN_TEST(test_stop_succeeds_and_recovers_bias_for_clean_circle);
    RUN_TEST(test_stop_recovers_axis_scale_for_elongated_ellipse);
    RUN_TEST(test_stop_noop_when_not_collecting);
    RUN_TEST(test_cancel_resets_to_idle);
    RUN_TEST(test_coverage_fraction_tracks_visited_sectors);
    RUN_TEST(test_span_deg_full_circle);
    RUN_TEST(test_span_deg_partial_arc);
    RUN_TEST(test_span_deg_zero_when_no_samples);
    return UNITY_END();
}
