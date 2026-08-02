#include <unity.h>
#include "ImuAngleMath.h"

using namespace ImuAngleMath;

void setUp(void) {}
void tearDown(void) {}

void test_wrap360_basic(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, wrap360(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 359.0, wrap360(-1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, wrap360(361.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, wrap360(360.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 180.0, wrap360(-180.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, wrap360(720.0));
}

void test_wrap180_basic(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, wrap180(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 180.0, wrap180(180.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 180.0, wrap180(-180.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.0, wrap180(359.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, wrap180(-359.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -90.0, wrap180(270.0));
}

void test_shortest_diff(void)
{
    // Crossing the 359/0 wrap - this is the case plain subtraction gets
    // wrong (359 to 1 the "long way" is 358, the short way is +2).
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0, shortestDiff(359.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -2.0, shortestDiff(1.0, 359.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, shortestDiff(350.0, 0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, shortestDiff(45.0, 45.0));
    // Exactly opposite - either sign is a valid shortest path; wrap180's
    // convention (fmod pushes to +180.0, verified in test_wrap180_basic)
    // makes this +180, not -180.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 180.0, shortestDiff(0.0, 180.0));
}

void test_circular_interpolate(void)
{
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, circularInterpolate(359.0, 1.0, 0.5));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 359.0, circularInterpolate(359.0, 1.0, 0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, circularInterpolate(359.0, 1.0, 1.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, circularInterpolate(0.0, 180.0, 0.5));
}

void test_circular_weighted_average(void)
{
    // Straddling the wrap, equal weight - should average to 0, not 180
    // (which is what a naive arithmetic mean of 359 and 1 would give).
    // Compared via shortestDiff, not raw equality: the true result sits
    // exactly on the 0/360 boundary, and which side floating point
    // rounding lands on (0.0 vs 359.999999...) is not meaningful - only
    // the circular distance from the expected value is.
    double angles[] = {359.0, 1.0};
    double weights[] = {1.0, 1.0};
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, shortestDiff(0.0, circularWeightedAverage(angles, weights, 2)));

    // Unequal weight pulls the result toward the heavier sample.
    double angles2[] = {0.0, 90.0};
    double weights2[] = {3.0, 1.0};
    double result = circularWeightedAverage(angles2, weights2, 2);
    TEST_ASSERT_TRUE(result < 45.0);

    // n<=0 returns 0 rather than crashing/undefined.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, circularWeightedAverage(nullptr, nullptr, 0));
}

void test_circular_low_pass(void)
{
    // alpha=0 -> no movement
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 350.0, circularLowPass(350.0, 10.0, 0.0));
    // alpha=1 -> snaps straight to the new value
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, circularLowPass(350.0, 10.0, 1.0));
    // alpha=0.5 across the wrap (350 -> 10 is a 20-degree gap the short
    // way) should land at 0, not regress through 180.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, circularLowPass(350.0, 10.0, 0.5));
}

void test_unwrapped_accumulator_basic(void)
{
    ImuAngleMath::UnwrappedAccumulator acc;
    TEST_ASSERT_FALSE(acc.isInitialized());
    acc.reset(10.0);
    TEST_ASSERT_TRUE(acc.isInitialized());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, acc.value());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, acc.update(20.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, acc.update(30.0));
}

void test_unwrapped_accumulator_crosses_north(void)
{
    // A steady clockwise turn through north should accumulate past 360,
    // not snap back to a small number at the wrap.
    ImuAngleMath::UnwrappedAccumulator acc;
    acc.reset(350.0);
    acc.update(355.0);
    acc.update(0.0);
    double v = acc.update(5.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 365.0, v);
}

void test_unwrapped_accumulator_counterclockwise_through_north(void)
{
    ImuAngleMath::UnwrappedAccumulator acc;
    acc.reset(10.0);
    acc.update(5.0);
    acc.update(0.0);
    double v = acc.update(355.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -5.0, v);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_wrap360_basic);
    RUN_TEST(test_wrap180_basic);
    RUN_TEST(test_shortest_diff);
    RUN_TEST(test_circular_interpolate);
    RUN_TEST(test_circular_weighted_average);
    RUN_TEST(test_circular_low_pass);
    RUN_TEST(test_unwrapped_accumulator_basic);
    RUN_TEST(test_unwrapped_accumulator_crosses_north);
    RUN_TEST(test_unwrapped_accumulator_counterclockwise_through_north);
    return UNITY_END();
}
