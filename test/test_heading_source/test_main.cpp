#include <unity.h>
#include <math.h>
#include "ImuHeadingSource.h"
#include "ImuQuaternion.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

// --- DmpValidator ---

void test_dmp_validator_flags_initializing_at_startup(void)
{
    DmpValidator v;
    Quaternion q; // identity, valid unit
    uint32_t flags = v.validate(q, true, 0, 0, false, 0, 0.02);
    TEST_ASSERT_TRUE((flags & HR_INITIALIZING) != 0);
}

void test_dmp_validator_clears_after_convergence_period(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 1000;
    cfg.minConsecutiveValidSamples = 3;
    Quaternion q;
    uint32_t flags = HR_NONE;
    // Feed enough valid samples, past the startup window.
    for (int i = 0; i < 10; i++)
        flags = v.validate(q, true, 0, 2000, false, 2000, 0.02, cfg);
    TEST_ASSERT_EQUAL_UINT32(HR_NONE, flags);
}

void test_dmp_validator_flags_stale(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 0;
    cfg.minConsecutiveValidSamples = 1;
    Quaternion q;
    v.validate(q, true, 0, 0, false, 5000, 0.02, cfg); // prime past init
    uint32_t flags = v.validate(q, false, 1000, 0, false, 5000, 0.02, cfg); // stale, age > maxSampleAgeMs
    TEST_ASSERT_TRUE((flags & HR_DMP_STALE) != 0);
}

void test_dmp_validator_flags_bad_quaternion(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 0;
    Quaternion bad;
    bad.w = 5.0; // way off unit norm
    uint32_t flags = v.validate(bad, true, 0, 0, false, 5000, 0.02, cfg);
    TEST_ASSERT_TRUE((flags & HR_BAD_QUATERNION) != 0);
}

void test_dmp_validator_flags_compass_disagreement(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 0;
    cfg.minConsecutiveValidSamples = 1;
    Quaternion q = ImuQuaternion::fromEuler(0, 0, 0); // heading 0
    uint32_t flags = v.validate(q, true, 0, 90.0, true, 5000, 0.02, cfg); // compass says 90, way off
    TEST_ASSERT_TRUE((flags & HR_DMP_COMPASS_DISAGREE) != 0);
}

void test_dmp_validator_agreement_with_compass_is_fine(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 0;
    cfg.minConsecutiveValidSamples = 1;
    Quaternion q = ImuQuaternion::fromEuler(0, 0, 0);
    uint32_t flags = v.validate(q, true, 0, 2.0, true, 5000, 0.02, cfg); // close agreement
    TEST_ASSERT_FALSE((flags & HR_DMP_COMPASS_DISAGREE) != 0);
}

void test_dmp_validator_flags_sudden_jump(void)
{
    DmpValidator v;
    DmpValidationConfig cfg;
    cfg.startupConvergenceMs = 0;
    cfg.minConsecutiveValidSamples = 1;
    cfg.maxJumpDegPerSec = 90.0;
    Quaternion q0 = ImuQuaternion::fromEuler(0, 0, 0);
    v.validate(q0, true, 0, 0, false, 5000, 0.02, cfg);
    Quaternion q1 = ImuQuaternion::fromEuler(0, 0, 170.0 * M_PI / 180.0); // huge jump in one 0.02s step
    uint32_t flags = v.validate(q1, true, 0, 0, false, 5000, 0.02, cfg);
    TEST_ASSERT_TRUE((flags & HR_SUDDEN_JUMP) != 0);
}

// --- HeadingSourceSelector ---

void test_selector_auto_prefers_fusion_over_dmp_over_compass(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusion{true, 10.0, HeadingQuality::Good};
    SourceCandidate dmp{true, 20.0, HeadingQuality::Good};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::Auto, fusion, dmp, compass, 0);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)r.source);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, r.headingDeg);
}

void test_selector_auto_falls_back_to_dmp_then_compass(void)
{
    HeadingSourceSelector sel1;
    SourceCandidate fusionInvalid{false, 0, HeadingQuality::Invalid};
    SourceCandidate dmp{true, 20.0, HeadingQuality::Good};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r1 = sel1.update(HeadingSourceMode::Auto, fusionInvalid, dmp, compass, 0);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::Dmp, (int)r1.source);

    HeadingSourceSelector sel2;
    SourceCandidate dmpInvalid{false, 0, HeadingQuality::Invalid};
    auto r2 = sel2.update(HeadingSourceMode::Auto, fusionInvalid, dmpInvalid, compass, 0);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)r2.source);
}

void test_selector_all_invalid_reports_invalid(void)
{
    HeadingSourceSelector sel;
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    auto r = sel.update(HeadingSourceMode::Auto, none, none, none, 0);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::None, (int)r.source);
}

void test_selector_explicit_mode_ignores_other_sources(void)
{
    HeadingSourceSelector sel;
    SourceCandidate fusion{true, 10.0, HeadingQuality::Good};
    SourceCandidate dmp{true, 20.0, HeadingQuality::Good};
    SourceCandidate compass{true, 30.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::SoftwareCompass, fusion, dmp, compass, 0);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)r.source);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, r.headingDeg);
}

void test_selector_explicit_mode_invalid_when_desired_source_unavailable(void)
{
    HeadingSourceSelector sel;
    SourceCandidate dmpInvalid{false, 0, HeadingQuality::Invalid};
    SourceCandidate other{true, 10.0, HeadingQuality::Good};
    auto r = sel.update(HeadingSourceMode::Dmp, other, dmpInvalid, other, 0);
    TEST_ASSERT_FALSE(r.valid);
}

void test_selector_first_valid_source_snaps_no_blend(void)
{
    HeadingSourceSelector sel;
    SourceCandidate compass{true, 123.0, HeadingQuality::Good};
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    auto r = sel.update(HeadingSourceMode::Auto, none, none, compass, 0, 1000);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 123.0, r.headingDeg);
}

void test_selector_source_change_blends_over_transition_duration(void)
{
    HeadingSourceSelector sel;
    SourceCandidate compass{true, 0.0, HeadingQuality::Good};
    SourceCandidate none{false, 0, HeadingQuality::Invalid};
    // Start on compass at heading 0.
    sel.update(HeadingSourceMode::Auto, none, none, compass, 0, 1000);

    // Fusion becomes available at heading 90 - should trigger a blend,
    // not an instant jump.
    SourceCandidate fusion{true, 90.0, HeadingQuality::Good};
    auto rStart = sel.update(HeadingSourceMode::Auto, fusion, none, compass, 0, 1000);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)rStart.source);
    // Right at the transition start, output should still be near the old
    // value (0), not already at 90.
    TEST_ASSERT_TRUE(fabs(ImuAngleMath::shortestDiff(0.0, rStart.headingDeg)) < 10.0);
    TEST_ASSERT_TRUE(sel.hasRecentTransition());
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareCompass, (int)sel.lastTransition().from);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)sel.lastTransition().to);

    // Halfway through the blend duration, should be roughly halfway
    // between 0 and 90.
    auto rMid = sel.update(HeadingSourceMode::Auto, fusion, none, compass, 500, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(15.0, 45.0, rMid.headingDeg);

    // After the full duration, should have fully arrived at the new
    // source's value.
    auto rEnd = sel.update(HeadingSourceMode::Auto, fusion, none, compass, 1001, 1000);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, rEnd.headingDeg);
    TEST_ASSERT_EQUAL_INT((int)HeadingSource::SoftwareFusion, (int)rEnd.source);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_dmp_validator_flags_initializing_at_startup);
    RUN_TEST(test_dmp_validator_clears_after_convergence_period);
    RUN_TEST(test_dmp_validator_flags_stale);
    RUN_TEST(test_dmp_validator_flags_bad_quaternion);
    RUN_TEST(test_dmp_validator_flags_compass_disagreement);
    RUN_TEST(test_dmp_validator_agreement_with_compass_is_fine);
    RUN_TEST(test_dmp_validator_flags_sudden_jump);
    RUN_TEST(test_selector_auto_prefers_fusion_over_dmp_over_compass);
    RUN_TEST(test_selector_auto_falls_back_to_dmp_then_compass);
    RUN_TEST(test_selector_all_invalid_reports_invalid);
    RUN_TEST(test_selector_explicit_mode_ignores_other_sources);
    RUN_TEST(test_selector_explicit_mode_invalid_when_desired_source_unavailable);
    RUN_TEST(test_selector_first_valid_source_snaps_no_blend);
    RUN_TEST(test_selector_source_change_blends_over_transition_duration);
    return UNITY_END();
}
