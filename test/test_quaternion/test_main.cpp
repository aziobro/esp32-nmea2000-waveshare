#include <unity.h>
#include <math.h>
#include "ImuQuaternion.h"

void setUp(void) {}
void tearDown(void) {}

void test_identity_quaternion_is_valid_unit(void)
{
    Quaternion q;
    TEST_ASSERT_TRUE(ImuQuaternion::isFinite(q));
    TEST_ASSERT_TRUE(ImuQuaternion::isValidUnit(q));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, ImuQuaternion::norm(q));
}

void test_normalize_scales_to_unit(void)
{
    Quaternion q;
    q.w = 2;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    Quaternion n = ImuQuaternion::normalize(q);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, ImuQuaternion::norm(n));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, n.w);
}

void test_normalize_degenerate_returns_identity(void)
{
    Quaternion q;
    q.w = 0;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    Quaternion n = ImuQuaternion::normalize(q);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, n.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, n.x);
}

void test_is_finite_rejects_nan(void)
{
    Quaternion q;
    q.x = NAN;
    TEST_ASSERT_FALSE(ImuQuaternion::isFinite(q));
    TEST_ASSERT_FALSE(ImuQuaternion::isValidUnit(q));
}

void test_is_finite_rejects_inf(void)
{
    Quaternion q;
    q.y = INFINITY;
    TEST_ASSERT_FALSE(ImuQuaternion::isFinite(q));
}

void test_is_valid_unit_rejects_bad_norm(void)
{
    Quaternion q;
    q.w = 3.0; // norm 3, way off unit
    q.x = 0;
    q.y = 0;
    q.z = 0;
    TEST_ASSERT_FALSE(ImuQuaternion::isValidUnit(q, 0.05));
}

void test_is_valid_unit_accepts_within_tolerance(void)
{
    Quaternion q;
    q.w = 1.02; // small FIFO-noise-scale deviation
    q.x = 0;
    q.y = 0;
    q.z = 0;
    TEST_ASSERT_TRUE(ImuQuaternion::isValidUnit(q, 0.05));
}

static void assertAngleEqual(double expectedRad, double actualRad, double tolRad = 1e-6)
{
    double diffDeg = fabs(expectedRad - actualRad) * (180.0 / M_PI);
    // handle wrap for angles near +/-180
    if (diffDeg > 180.0)
        diffDeg = 360.0 - diffDeg;
    TEST_ASSERT_DOUBLE_WITHIN(tolRad * (180.0 / M_PI), 0.0, diffDeg);
}

void test_euler_round_trip_various_angles(void)
{
    // Avoid pitch near +/-90 (gimbal lock - roll/yaw become ambiguous,
    // not a meaningful round-trip case).
    double rolls[] = {0, 10, -10, 45, -45, 89, -89};
    double pitches[] = {0, 10, -10, 30, -30, 60, -60};
    double yaws[] = {0, 45, 90, 135, 180, 225, 270, 315, 5, 355};

    for (double r : rolls)
    {
        for (double p : pitches)
        {
            for (double y : yaws)
            {
                double rRad = r * M_PI / 180.0;
                double pRad = p * M_PI / 180.0;
                double yRad = y * M_PI / 180.0;
                Quaternion q = ImuQuaternion::fromEuler(rRad, pRad, yRad);
                TEST_ASSERT_TRUE(ImuQuaternion::isValidUnit(q, 1e-6));
                double outR, outP, outY;
                ImuQuaternion::toEuler(q, outR, outP, outY);
                assertAngleEqual(rRad, outR);
                assertAngleEqual(pRad, outP);
                assertAngleEqual(yRad, outY);
            }
        }
    }
}

void test_yaw_difference_zero_for_same_heading(void)
{
    Quaternion a = ImuQuaternion::fromEuler(0, 0, 1.0);
    Quaternion b = ImuQuaternion::fromEuler(0.1, -0.1, 1.0); // different roll/pitch, same yaw
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.0, ImuQuaternion::yawDifferenceDeg(a, b));
}

void test_yaw_difference_across_wrap(void)
{
    Quaternion a = ImuQuaternion::fromEuler(0, 0, 359.0 * M_PI / 180.0);
    Quaternion b = ImuQuaternion::fromEuler(0, 0, 1.0 * M_PI / 180.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, 2.0, fabs(ImuQuaternion::yawDifferenceDeg(a, b)));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_identity_quaternion_is_valid_unit);
    RUN_TEST(test_normalize_scales_to_unit);
    RUN_TEST(test_normalize_degenerate_returns_identity);
    RUN_TEST(test_is_finite_rejects_nan);
    RUN_TEST(test_is_finite_rejects_inf);
    RUN_TEST(test_is_valid_unit_rejects_bad_norm);
    RUN_TEST(test_is_valid_unit_accepts_within_tolerance);
    RUN_TEST(test_euler_round_trip_various_angles);
    RUN_TEST(test_yaw_difference_zero_for_same_heading);
    RUN_TEST(test_yaw_difference_across_wrap);
    return UNITY_END();
}
