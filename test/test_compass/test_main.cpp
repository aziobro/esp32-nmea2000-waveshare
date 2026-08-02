#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "ImuCompass.h"
#include "ImuCalibration.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

// --- Synthetic magnetic field model, test-only ---
//
// Earth's field has a horizontal component (magnitude Bh, points at
// magnetic north) and a vertical/dip component (magnitude Bv, points
// down in the northern hemisphere). For a LEVEL boat at true heading H
// (0=north, clockwise-positive, matching the boat's own heading
// convention), the field expressed in boat frame (X=fwd,Y=starboard) is
// derived from first principles (rotating the fixed reference-frame field
// into the boat's heading-rotated frame): mx=Bh*cos(H), my=-Bh*sin(H),
// mz=Bv (see the derivation in the commit message / architecture doc).
//
// For a TILTED boat, the sensor reads that same level-frame field further
// rotated by the tilt. The composition order (pitch first, then roll) and
// signs here were derived to match what ImuCompass::rawHeadingDeg
// actually undoes - confirmed, not assumed, by the tilt-invariance tests
// below (they'd fail immediately if this were wrong).
static Vec3 syntheticLevelField(double trueHeadingDeg, double Bh, double Bv)
{
    double H = trueHeadingDeg * (M_PI / 180.0);
    return Vec3(Bh * cos(H), -Bh * sin(H), Bv);
}

static Vec3 applyTilt(const Vec3 &level, double rollRad, double pitchRad)
{
    // Pitch first: rotate around Y by -pitch.
    double cp = cos(pitchRad), sp = sin(pitchRad);
    double x1 = level.x * cp - level.z * sp;
    double y1 = level.y;
    double z1 = level.x * sp + level.z * cp;
    // Then roll: rotate around X by -roll.
    double cr = cos(rollRad), sr = sin(rollRad);
    double x2 = x1;
    double y2 = cr * y1 + sr * z1;
    double z2 = -sr * y1 + cr * z1;
    return Vec3(x2, y2, z2);
}

static const double BH = 22.0; // arbitrary horizontal field magnitude, uT-ish
static const double BV = 45.0; // arbitrary vertical/dip component, northern hemisphere

// Reference heading output for H=0, level - every other case is checked
// relative to this via ImuAngleMath::shortestDiff, since the formula's
// absolute sign/zero-point convention (dependent on the real chip's
// unknowable-without-hardware axis handedness) isn't itself under test
// here - see icmHdgInv in the existing config. What IS under test: does
// the output change correctly and consistently as heading/roll/pitch
// change, and does tilt compensation actually remove tilt-induced error.
static double headingFor(double trueHeadingDeg, double rollDeg, double pitchDeg)
{
    Vec3 level = syntheticLevelField(trueHeadingDeg, BH, BV);
    Vec3 tilted = applyTilt(level, rollDeg * (M_PI / 180.0), pitchDeg * (M_PI / 180.0));
    return ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(tilted, rollDeg * (M_PI / 180.0), pitchDeg * (M_PI / 180.0)));
}

void test_cardinal_headings_level_are_self_consistent(void)
{
    double h0 = headingFor(0, 0, 0);
    double h90 = headingFor(90, 0, 0);
    double h180 = headingFor(180, 0, 0);
    double h270 = headingFor(270, 0, 0);
    // Each 90-degree true-heading step must produce a 90-degree step in
    // the computed heading, in a consistent rotational direction (the
    // absolute direction/sign depends on chip handedness - icmHdgInv - so
    // this checks magnitude and consistency, not a specific sign).
    double d1 = fabs(ImuAngleMath::shortestDiff(h0, h90));
    double d2 = fabs(ImuAngleMath::shortestDiff(h90, h180));
    double d3 = fabs(ImuAngleMath::shortestDiff(h180, h270));
    double d4 = fabs(ImuAngleMath::shortestDiff(h270, h0 + 360));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, d1);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, d2);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, d3);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 90.0, d4);
}

void test_north_south_are_opposite(void)
{
    double h0 = headingFor(0, 0, 0);
    double h180 = headingFor(180, 0, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 180.0, fabs(ImuAngleMath::shortestDiff(h0, h180)));
}

// The actual point of tilt compensation: heeling/pitching must NOT change
// the computed heading for a fixed true heading, across a range of
// cardinal headings and tilt angles.
void test_tilt_compensation_roll(void)
{
    double cardinals[] = {0, 90, 180, 270, 45, 135, 225, 315};
    double rolls[] = {-30, -20, -10, 10, 20, 30};
    for (double H : cardinals)
    {
        double level = headingFor(H, 0, 0);
        for (double r : rolls)
        {
            double tilted = headingFor(H, r, 0);
            double diff = fabs(ImuAngleMath::shortestDiff(level, tilted));
            char msg[96];
            snprintf(msg, sizeof(msg), "H=%.0f roll=%.0f: level=%.3f tilted=%.3f diff=%.6f", H, r, level, tilted, diff);
            TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, 0.0, diff, msg);
        }
    }
}

void test_tilt_compensation_pitch(void)
{
    double cardinals[] = {0, 90, 180, 270, 45, 135, 225, 315};
    double pitches[] = {-30, -20, -10, 10, 20, 30};
    for (double H : cardinals)
    {
        double level = headingFor(H, 0, 0);
        for (double p : pitches)
        {
            double tilted = headingFor(H, 0, p);
            double diff = fabs(ImuAngleMath::shortestDiff(level, tilted));
            char msg[96];
            snprintf(msg, sizeof(msg), "H=%.0f pitch=%.0f: level=%.3f tilted=%.3f diff=%.6f", H, p, level, tilted, diff);
            TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, 0.0, diff, msg);
        }
    }
}

void test_tilt_compensation_combined_roll_and_pitch(void)
{
    double cardinals[] = {0, 90, 180, 270, 33, 217};
    double angles[] = {-25, -15, 15, 25};
    for (double H : cardinals)
    {
        double level = headingFor(H, 0, 0);
        for (double r : angles)
        {
            for (double p : angles)
            {
                double tilted = headingFor(H, r, p);
                double diff = fabs(ImuAngleMath::shortestDiff(level, tilted));
                char msg[96];
                snprintf(msg, sizeof(msg), "H=%.0f roll=%.0f pitch=%.0f: diff=%.6f", H, r, p, diff);
                TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, 0.0, diff, msg);
            }
        }
    }
}

void test_wrap_at_north(void)
{
    // 359 and 1 degrees true heading should produce outputs 2 degrees
    // apart the short way, not 358 - exercises the same wrap the formula
    // must get right in practice.
    double h359 = headingFor(359, 0, 0);
    double h1 = headingFor(1, 0, 0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.0, fabs(ImuAngleMath::shortestDiff(h359, h1)));
}

// Known hard-iron offset: applying a calibration that removes a bias
// injected into the raw reading must recover the same heading as the
// uncorrupted case.
void test_hard_iron_correction_recovers_heading(void)
{
    double H = 60.0;
    Vec3 level = syntheticLevelField(H, BH, BV);
    Vec3 clean = applyTilt(level, 0, 0);
    double cleanHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(clean, 0, 0));

    Vec3 biasedRaw(clean.x + 8.0, clean.y - 5.0, clean.z + 3.0);
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magBias[0] = 8.0;
    cal.magBias[1] = -5.0;
    cal.magBias[2] = 3.0;
    Vec3 corrected = ImuCalibrationOps::applyMag(biasedRaw, cal);
    double correctedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(corrected, 0, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(cleanHeading, correctedHeading)));

    // Sanity: the UNcorrected biased reading should generally NOT give
    // the same heading (proves the test would actually catch a broken
    // calibration, not just trivially pass).
    double uncorrectedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(biasedRaw, 0, 0));
    TEST_ASSERT_TRUE(fabs(ImuAngleMath::shortestDiff(cleanHeading, uncorrectedHeading)) > 1.0);
}

// Known unequal axis scale (soft-iron, diagonal case): a calibration that
// un-scales a distorted reading must recover the same heading.
void test_unequal_scale_correction_recovers_heading(void)
{
    double H = 200.0;
    Vec3 clean = syntheticLevelField(H, BH, BV);
    double cleanHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(clean, 0, 0));

    // Distort: X axis reads at 1.4x, Y at 0.7x (elliptical distortion).
    Vec3 distorted(clean.x * 1.4, clean.y * 0.7, clean.z);
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magMatrix[0][0] = 1.0 / 1.4;
    cal.magMatrix[1][1] = 1.0 / 0.7;
    Vec3 corrected = ImuCalibrationOps::applyMag(distorted, cal);
    double correctedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(corrected, 0, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(cleanHeading, correctedHeading)));
}

// Known full soft-iron matrix (non-diagonal / rotated ellipse case).
void test_soft_iron_matrix_correction_recovers_heading(void)
{
    double H = 310.0;
    Vec3 clean = syntheticLevelField(H, BH, BV);
    double cleanHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(clean, 0, 0));

    // A distortion matrix with cross-axis coupling.
    double D[2][2] = {{1.2, 0.3}, {0.1, 0.9}};
    Vec3 distorted(D[0][0] * clean.x + D[0][1] * clean.y, D[1][0] * clean.x + D[1][1] * clean.y, clean.z);

    // Invert the 2x2 distortion (closed form) to build the calibration matrix.
    double det = D[0][0] * D[1][1] - D[0][1] * D[1][0];
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magMatrix[0][0] = D[1][1] / det;
    cal.magMatrix[0][1] = -D[0][1] / det;
    cal.magMatrix[1][0] = -D[1][0] / det;
    cal.magMatrix[1][1] = D[0][0] / det;

    Vec3 corrected = ImuCalibrationOps::applyMag(distorted, cal);
    double correctedHeading = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(corrected, 0, 0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, fabs(ImuAngleMath::shortestDiff(cleanHeading, correctedHeading)));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_cardinal_headings_level_are_self_consistent);
    RUN_TEST(test_north_south_are_opposite);
    RUN_TEST(test_tilt_compensation_roll);
    RUN_TEST(test_tilt_compensation_pitch);
    RUN_TEST(test_tilt_compensation_combined_roll_and_pitch);
    RUN_TEST(test_wrap_at_north);
    RUN_TEST(test_hard_iron_correction_recovers_heading);
    RUN_TEST(test_unequal_scale_correction_recovers_heading);
    RUN_TEST(test_soft_iron_matrix_correction_recovers_heading);
    return UNITY_END();
}
