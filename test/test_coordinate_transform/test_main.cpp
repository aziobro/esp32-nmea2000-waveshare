#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "ImuCoordinateTransform.h"
#include "ImuQuaternion.h"

void setUp(void) {}
void tearDown(void) {}

static void assertVecEqual(const Vec3 &expected, const Vec3 &actual, double tol = 1e-9)
{
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.x, actual.x);
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.y, actual.y);
    TEST_ASSERT_DOUBLE_WITHIN(tol, expected.z, actual.z);
}

// v' = q * v * conj(q), treating v as a pure (w=0) quaternion - the
// standard quaternion sandwich rotation, matching the convention toEuler/
// fromEuler and quaternionFor/rotateDmpQuaternion all share.
static Vec3 rotateVectorByQuaternion(const Quaternion &q, const Vec3 &v)
{
    Quaternion vq;
    vq.w = 0;
    vq.x = v.x;
    vq.y = v.y;
    vq.z = v.z;
    Quaternion r = ImuQuaternion::multiply(ImuQuaternion::multiply(q, vq), ImuQuaternion::conjugate(q));
    return Vec3(r.x, r.y, r.z);
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

// quaternionFor(o), sandwich-applied to an arbitrary vector, must produce
// exactly what matrixFor(o).apply() produces - this is the real safety
// net for the matrix-to-quaternion conversion (Shepperd's method has
// several branches; this proves all of them, not just the one the other
// hand-picked test vectors happen to hit).
void test_quaternionFor_matches_matrixFor_for_all_24_orientations(void)
{
    Vec3 probe(0.4, -0.6, 0.8); // asymmetric, no zero components - exercises every matrix entry
    for (int i = 0; i < 24; i++)
    {
        MountOrientation o = static_cast<MountOrientation>(i);
        Vec3 viaMatrix = ImuCoordinateTransform::matrixFor(o).apply(probe);
        Vec3 viaQuat = rotateVectorByQuaternion(ImuCoordinateTransform::quaternionFor(o), probe);
        char msg[64];
        snprintf(msg, sizeof(msg), "orientation index %d: quaternion form disagrees with matrix form", i);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, viaMatrix.x, viaQuat.x, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, viaMatrix.y, viaQuat.y, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, viaMatrix.z, viaQuat.z, msg);
    }
}

void test_quaternionFor_forward_is_identity(void)
{
    Quaternion q = ImuCoordinateTransform::quaternionFor(MountOrientation::Forward);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, q.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, q.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, q.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, q.z);
}

// Forward is the identity transform and must reproduce the pre-fix
// behavior exactly - a currently-deployed unit using the default
// orientation must see zero change from this fix.
void test_rotateDmpQuaternion_forward_is_noop(void)
{
    Quaternion dmp = ImuQuaternion::fromEuler(0.3, -0.2, 1.1); // arbitrary non-trivial attitude
    Quaternion out = ImuCoordinateTransform::rotateDmpQuaternion(dmp, MountOrientation::Forward);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, dmp.w, out.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, dmp.x, out.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, dmp.y, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, dmp.z, out.z);
}

// The defining property, tested end-to-end rather than by re-deriving the
// same algebra: dmpQuat maps a SENSOR-frame vector to its WORLD-frame
// representation (v_world = dmp * v_sensor); rotateDmpQuaternion's result
// must map the corresponding BOAT-frame vector to that exact same
// WORLD-frame vector. Framed as an inverse problem (given an arbitrary
// world vector, recover the body-frame vector two independent ways) so it
// never needs to convert a world-frame quantity through matrixFor, which
// only relates SENSOR and BOAT frames to each other, not to world frame -
// an earlier version of this test made exactly that mistake and produced
// a false failure. If the multiply/conjugate order in rotateDmpQuaternion
// were wrong, this would fail for any non-Forward orientation - it would
// NOT coincidentally pass, unlike a test that just re-checks the same
// formula.
void test_rotateDmpQuaternion_consistent_with_matrix_for_all_24_orientations(void)
{
    Quaternion dmp = ImuQuaternion::fromEuler(0.25, 0.4, -0.6); // arbitrary non-trivial attitude
    Vec3 vWorld(0.4, -0.6, 0.8); // arbitrary, no zero components
    for (int i = 0; i < 24; i++)
    {
        MountOrientation o = static_cast<MountOrientation>(i);
        Mat3 m = ImuCoordinateTransform::matrixFor(o);
        Quaternion boatQuat = ImuCoordinateTransform::rotateDmpQuaternion(dmp, o);

        // Path 1: recover the sensor-frame vector via dmp's inverse, then
        // convert sensor->boat with the orientation matrix directly.
        Vec3 vSensor = rotateVectorByQuaternion(ImuQuaternion::conjugate(dmp), vWorld);
        Vec3 expectedBoat = m.apply(vSensor);

        // Path 2: recover the boat-frame vector directly via boatQuat's
        // inverse - no matrixFor involved at all.
        Vec3 actualBoat = rotateVectorByQuaternion(ImuQuaternion::conjugate(boatQuat), vWorld);

        char msg[96];
        snprintf(msg, sizeof(msg), "orientation index %d: boat-frame quaternion disagrees with sensor-frame + matrix path", i);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, expectedBoat.x, actualBoat.x, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, expectedBoat.y, actualBoat.y, msg);
        TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-9, expectedBoat.z, actualBoat.z, msg);
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
    RUN_TEST(test_quaternionFor_matches_matrixFor_for_all_24_orientations);
    RUN_TEST(test_quaternionFor_forward_is_identity);
    RUN_TEST(test_rotateDmpQuaternion_forward_is_noop);
    RUN_TEST(test_rotateDmpQuaternion_consistent_with_matrix_for_all_24_orientations);
    return UNITY_END();
}
