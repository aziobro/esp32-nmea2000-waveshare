#include <unity.h>
#include <math.h>
#include "ImuHeadingSource.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

void test_selector_auto_prefers_fusion_over_compass(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusion{true, 10.0, HeadingQuality::Good};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::Auto, fusion, compass, 0);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)r.source);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, r.headingDeg);
}

void test_selector_auto_falls_back_to_compass(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusionInvalid{false, 0, HeadingQuality::Invalid};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::Auto, fusionInvalid, compass, 0);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)r.source);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, r.headingDeg);
}

void test_selector_all_invalid_reports_invalid(void)
{
    HeadingSourceSelector sel;
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    auto r = sel.update(HeadingSourceMode::Auto, none, none, 0);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::None, (int)r.source);
}

void test_selector_explicit_mode_ignores_other_sources(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusion{true, 10.0, HeadingQuality::Good};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::SoftwareCompass, fusion, compass, 0);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)r.source);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, r.headingDeg);
}

void test_selector_explicit_mode_invalid_when_desired_source_unavailable(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusionInvalid{false, 0, HeadingQuality::Invalid};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::SoftwareFusion, fusionInvalid, compass, 0);
    TEST_ASSERT_FALSE(r.valid);
}

void test_selector_first_valid_source_snaps_no_blend(void)
{
    HeadingSourceSelector sel;
    SourceCandidate compass{true, 123.0, HeadingQuality::Good};
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    auto r = sel.update(HeadingSourceMode::Auto, none, compass, 0, 1000);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 123.0, r.headingDeg);
}

void test_selector_source_change_blends_over_transition_duration(void)
{
    HeadingSourceSelector sel;
    SourceCandidate compass{true, 0.0, HeadingQuality::Good};
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    sel.update(HeadingSourceMode::Auto, none, compass, 0, 1000);

    SourceCandidate fusion{true, 90.0, HeadingQuality::Good};
    auto rStart = sel.update(HeadingSourceMode::Auto, fusion, compass, 0, 1000);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)rStart.source);
    TEST_ASSERT_TRUE(fabs(ImuAngleMath::shortestDiff(0.0, rStart.headingDeg)) < 10.0);
    TEST_ASSERT_TRUE(sel.hasRecentTransition());
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)sel.lastTransition().from);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)sel.lastTransition().to);

    auto rMid = sel.update(HeadingSourceMode::Auto, fusion, compass, 500, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(15.0, 45.0, rMid.headingDeg);

    auto rEnd = sel.update(HeadingSourceMode::Auto, fusion, compass, 1001, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, rEnd.headingDeg);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)rEnd.source);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_selector_auto_prefers_fusion_over_compass);
    RUN_TEST(test_selector_auto_falls_back_to_compass);
    RUN_TEST(test_selector_all_invalid_reports_invalid);
    RUN_TEST(test_selector_explicit_mode_ignores_other_sources);
    RUN_TEST(test_selector_explicit_mode_invalid_when_desired_source_unavailable);
    RUN_TEST(test_selector_first_valid_source_snaps_no_blend);
    RUN_TEST(test_selector_source_change_blends_over_transition_duration);
    return UNITY_END();
}
