#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "ImuCoordinateTransform.h"

void setUp(void) {}
void tearDown(void) {}

static void assertVecEqual(const Vec3 &expected, const Vec3 &actual, double tol = 1e-9)
{
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.x, actual.x);
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.y, actual.y);
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.z, actual.z);
}

void test_forward_is_identity(void)
{
    Vec3 v(1.0, 2.0, 3.0);
    Vec3 out = ImuCoordinateTransform::toBoatFrame(v, MountOrientation::Forward);
    assertVecEqual(v, out);
}

// The four required flat-mount orientations, hand-derived against the
// boat's own heading convention (rotating the sensor 90 degrees clockwise
// as viewed from above turns "reads as forward" into "reads as
// starboard") - see ImuCoordinateTransform.cpp's comment for the full
// derivation. These are the ones the project spec calls out as the
// minimum required support, so they get exact numeric checks, not just
// general-validity checks.
void test_starboard_orientation(void)
{
    // Sensor's own +X (which reads "forward" when mounted Forward) should
    // read as boat +Y (starboard) when mounted Starboard.
    Vec3 out = ImuCoordinateTransform::toBoatFrame(Vec3(1, 0, 0), MountOrientation::Starboard);
    assertVecEqual(Vec3(0, 1, 0), out);
    // Down stays down regardless of horizontal facing.
    Vec3 down = ImuCoordinateTransform::toBoatFrame(Vec3(0, 0, 1), MountOrientation::Starboard);
    assertVecEqual(Vec3(0, 0, 1), down);
}

void test_aft_orientation(void)
{
    Vec3 out = ImuCoordinateTransform::toBoatFrame(Vec3(1, 0, 0), MountOrientation::Aft);
    assertVecEqual(Vec3(-1, 0, 0), out);
}

void test_port_orientation(void)
{
    Vec3 out = ImuCoordinateTransform::toBoatFrame(Vec3(1, 0, 0), MountOrientation::Port);
    assertVecEqual(Vec3(0, -1, 0), out);
}

// Applying Starboard four times should return to Forward (four 90-degree
// turns is a full circle) - a useful self-consistency check independent
// of the exact sign convention chosen.
void test_four_starboard_rotations_is_identity(void)
{
    Vec3 v(1, 0, 0);
    Mat3 s = ImuCoordinateTransform::matrixFor(MountOrientation::Starboard);
    Vec3 out = v;
    for (int i = 0; i < 4; i++)
        out = s.apply(out);
    assertVecEqual(v, out, 1e-9);
}

// Every one of the 24 orientations must be a mathematically valid
// rotation (orthogonal, determinant +1) - this is the general-correctness
// guarantee for the 20 orientations whose exact real-world mounting
// meaning is not yet confirmed against hardware (see header comment).
void test_all_24_orientations_are_valid_rotations(void)
{
    for (int i = 0; i < 24; i++)
    {
        Mat3 m = ImuCoordinateTransform::matrixFor(static_cast<MountOrientation>(i));
        char msg[64];
        snprintf(msg, sizeof(msg), "orientation index %d is not a valid rotation", i);
        TEST_ASSERT_TRUE_MESSAGE(ImuCoordinateTransform::isValidRotation(m), msg);
    }
}

// A rotation must preserve vector length.
void test_all_24_orientations_preserve_norm(void)
{
    Vec3 v(0.3, -0.7, 1.1);
    double expectedNorm = v.norm();
    for (int i = 0; i < 24; i++)
    {
        Vec3 out = ImuCoordinateTransform::toBoatFrame(v, static_cast<MountOrientation>(i));
        TEST_ASSERT_DOUBLE_WITHIN(1e-9, expectedNorm, out.norm());
    }
}

// transform then inverse-transform (transpose, valid for any orthogonal
// matrix) must round-trip exactly, for all 24 orientations.
void test_all_24_orientations_round_trip(void)
{
    Vec3 v(0.5, -1.2, 2.3);
    for (int i = 0; i < 24; i++)
    {
        Mat3 m = ImuCoordinateTransform::matrixFor(static_cast<MountOrientation>(i));
        Vec3 boatFrame = m.apply(v);
        Vec3 back = m.transposed().apply(boatFrame);
        char msg[64];
        snprintf(msg, sizeof(msg), "orientation index %d did not round-trip", i);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, v.x, back.x, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, v.y, back.y, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, v.z, back.z, msg);
    }
}

// All 24 must be distinct rotations (no accidental duplicate entries in
// the table) - checked by confirming no two map a fixed non-symmetric
// probe vector to the same result.
void test_all_24_orientations_are_distinct(void)
{
    Vec3 probe(1.0, 2.0, 3.0); // asymmetric on purpose, so any permutation/sign difference shows up
    Vec3 results[24];
    for (int i = 0; i < 24; i++)
        results[i] = ImuCoordinateTransform::toBoatFrame(probe, static_cast<MountOrientation>(i));
    for (int i = 0; i < 24; i++)
    {
        for (int j = i + 1; j < 24; j++)
        {
            double d = fabs(results[i].x - results[j].x) + fabs(results[i].y - results[j].y) + fabs(results[i].z - results[j].z);
            char msg[64];
            snprintf(msg, sizeof(msg), "orientations %d and %d produced the same result", i, j);
            TEST_ASSERT_TRUE_MESSAGE(d > 1e-6, msg);
        }
    }
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_forward_is_identity);
    RUN_TEST(test_starboard_orientation);
    RUN_TEST(test_aft_orientation);
    RUN_TEST(test_port_orientation);
    RUN_TEST(test_four_starboard_rotations_is_identity);
    RUN_TEST(test_all_24_orientations_are_valid_rotations);
    RUN_TEST(test_all_24_orientations_preserve_norm);
    RUN_TEST(test_all_24_orientations_round_trip);
    RUN_TEST(test_all_24_orientations_are_distinct);
    return UNITY_END();
}
