#pragma once
#include "ImuTypes.h"

/*
  Maps a vector measured in the ICM-20948's own local (sensor) frame into
  the boat frame (X=forward, Y=starboard, Z=down) for a given physical
  mounting orientation. Applied identically to accelerometer, gyroscope,
  and magnetometer readings - see doc/IcmHeadingArchitecture.md.

  MountOrientation::Forward is the identity transform and reproduces
  exactly what the pre-rewrite code always assumed (sensor axes already
  aligned with boat axes, with only the existing icmRollInv/icmPitchInv/
  icmHdgInv/icmRotInv sign flips available) - the currently-deployed unit's
  behavior doesn't change unless a different orientation is selected.

  Forward/Starboard/Aft/Port (flat, horizontal mounts - the minimum this
  project's spec requires) are hand-derived and individually verified
  against the boat's own heading convention (rotating the sensor 90
  degrees clockwise as viewed from above turns what used to read
  "forward" into "starboard", matching how a ship's heading itself
  increases clockwise). The remaining 20 (tilted/vertical bulkhead mounts,
  and every flat mount's upside-down variant) are generated exhaustively -
  every signed permutation of the three axes with determinant +1 is a
  valid 90-degree-step rotation, and there are exactly 24 of them (one per
  way to orient a cube); each is a verified-valid rotation (orthogonal,
  determinant +1, round-trips a vector through transform+inverse
  correctly, and is distinct from all 23 others - see
  test_coordinate_transform), but which exact enum value corresponds to
  which real physical mounting is NOT yet confirmed against real hardware
  and should be treated as provisional until the physical test procedure
  (doc/IcmPhysicalTestProcedure.md) confirms it.
*/

struct Mat3
{
    double m[3][3];

    // A default member initializer here would make Mat3 a non-aggregate
    // under some toolchains' C++ standard defaults (confirmed: the ESP32
    // build rejects brace-init literals like {{{0,-1,0},...}} with
    // "could not convert... to const Mat3" even though the same literals
    // compile fine under the native/desktop test env's newer default
    // standard) - an explicit constructor works identically everywhere.
    Mat3()
    {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
    }
    Mat3(double m00, double m01, double m02,
         double m10, double m11, double m12,
         double m20, double m21, double m22)
    {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    Vec3 apply(const Vec3 &v) const
    {
        return Vec3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }

    // Valid for any orthogonal matrix (all rotations here are) - transpose
    // is the inverse, far cheaper than a general matrix inverse.
    Mat3 transposed() const
    {
        Mat3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r.m[i][j] = m[j][i];
        return r;
    }

    Mat3 multiply(const Mat3 &o) const
    {
        Mat3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                double s = 0;
                for (int k = 0; k < 3; k++)
                    s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }

    double determinant() const
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    static Mat3 identity() { return Mat3(); }
};

namespace ImuCoordinateTransform
{
    // The signed rotation matrix for a given mounting orientation - sensor
    // frame in, boat frame out.
    Mat3 matrixFor(MountOrientation orientation);

    // Convenience: matrixFor(orientation).apply(sensorFrameVec)
    Vec3 toBoatFrame(const Vec3 &sensorFrameVec, MountOrientation orientation);

    // True if every one of the 24 orientation matrices is a valid rotation
    // (orthogonal, determinant +1) - used by tests, exposed here so the
    // check itself is testable/reusable rather than duplicated.
    bool isValidRotation(const Mat3 &m, double tolerance = 1e-9);
}
