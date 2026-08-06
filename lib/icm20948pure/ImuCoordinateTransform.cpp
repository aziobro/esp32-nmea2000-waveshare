#include "ImuCoordinateTransform.h"
#include "ImuQuaternion.h"
#include <math.h>

namespace
{
    // Every signed permutation matrix (row i has its one nonzero entry at
    // column perm[i], value sign[i]) with determinant +1 is a valid
    // 90-degree-step rotation, and there are exactly 24 of them - one for
    // each way to orient a cube. Different (permutation, sign) pairs are
    // trivially always different matrices, so generating all 24 this way
    // guarantees they're all distinct by construction (verified anyway in
    // test_coordinate_transform's distinctness test, but no clever
    // composition-order bugs are possible here the way there were with an
    // earlier draft that composed elementary rotations - two different
    // composition paths landed on the identical matrix).
    Mat3 buildSignedPermutation(const int perm[3], const int sign[3])
    {
        Mat3 m;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m.m[i][j] = (perm[i] == j) ? (double)sign[i] : 0.0;
        return m;
    }

    bool sameMatrix(const Mat3 &a, const Mat3 &b, double tol = 1e-9)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (fabs(a.m[i][j] - b.m[i][j]) > tol)
                    return false;
        return true;
    }

    // The four required flat-mount orientations, hand-derived against the
    // boat's own heading convention - see the header comment for the full
    // derivation. Matched against the generated pool below so they land
    // in their named enum slots (0-3); which of the remaining 20
    // (provisional, not yet physically verified) generated matrices ends
    // up in which of the other enum slots doesn't matter, since those
    // names aren't claimed to be physically confirmed yet.
    const Mat3 FORWARD = Mat3::identity();
    const Mat3 STARBOARD(0, -1, 0, 1, 0, 0, 0, 0, 1);
    const Mat3 AFT(-1, 0, 0, 0, -1, 0, 0, 0, 1);
    const Mat3 PORT(0, 1, 0, -1, 0, 0, 0, 0, 1);

    struct Table
    {
        Mat3 m[24];
        Table()
        {
            const int perms[6][3] = {
                {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
            Mat3 pool[24];
            bool used[24] = {false};
            int poolCount = 0;
            for (int p = 0; p < 6; p++)
            {
                for (int s0 = -1; s0 <= 1; s0 += 2)
                    for (int s1 = -1; s1 <= 1; s1 += 2)
                        for (int s2 = -1; s2 <= 1; s2 += 2)
                        {
                            int sign[3] = {s0, s1, s2};
                            Mat3 cand = buildSignedPermutation(perms[p], sign);
                            if (fabs(cand.determinant() - 1.0) < 1e-9)
                                pool[poolCount++] = cand;
                        }
            }
            // poolCount is always exactly 24 (6 permutations x 4 of 8 sign
            // combinations each have det +1).

            // Slot in the four hand-verified, named orientations first.
            const Mat3 *required[4] = {&FORWARD, &STARBOARD, &AFT, &PORT};
            for (int r = 0; r < 4; r++)
            {
                for (int i = 0; i < poolCount; i++)
                {
                    if (!used[i] && sameMatrix(pool[i], *required[r]))
                    {
                        m[r] = pool[i];
                        used[i] = true;
                        break;
                    }
                }
            }
            // Fill the remaining 20 slots with whatever's left, in
            // generation order - provisional pending physical
            // verification, see the header comment.
            int next = 4;
            for (int i = 0; i < poolCount && next < 24; i++)
            {
                if (!used[i])
                {
                    m[next++] = pool[i];
                    used[i] = true;
                }
            }
        }
    };

    const Table &table()
    {
        static const Table t;
        return t;
    }
}

namespace ImuCoordinateTransform
{
    Mat3 matrixFor(MountOrientation orientation)
    {
        int idx = static_cast<int>(orientation);
        if (idx < 0 || idx >= 24)
            return Mat3::identity();
        return table().m[idx];
    }

    Vec3 toBoatFrame(const Vec3 &sensorFrameVec, MountOrientation orientation)
    {
        return matrixFor(orientation).apply(sensorFrameVec);
    }

    // Standard robust rotation-matrix-to-quaternion conversion (Shepperd's
    // method - branches on the largest of {trace, m00, m11, m22} to avoid
    // dividing by a near-zero term, which a naive trace-only formula would
    // hit for some of the 180-degree-rotation orientations in this table,
    // e.g. Aft's trace is -1). Derived from, and therefore automatically
    // consistent with, the exact sandwich convention (v' = q*v*conj(q))
    // toEuler/fromEuler already use - test_coordinate_transform verifies
    // this by checking quaternionFor(o) sandwich-applied to sample vectors
    // reproduces matrixFor(o).apply() for all 24 orientations, rather than
    // trusting the derivation alone.
    Quaternion quaternionFor(MountOrientation orientation)
    {
        Mat3 m = matrixFor(orientation);
        double trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        Quaternion q;
        if (trace > 0)
        {
            double s = sqrt(trace + 1.0) * 2.0; // s = 4*qw
            q.w = 0.25 * s;
            q.x = (m.m[2][1] - m.m[1][2]) / s;
            q.y = (m.m[0][2] - m.m[2][0]) / s;
            q.z = (m.m[1][0] - m.m[0][1]) / s;
        }
        else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2])
        {
            double s = sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0; // s = 4*qx
            q.w = (m.m[2][1] - m.m[1][2]) / s;
            q.x = 0.25 * s;
            q.y = (m.m[0][1] + m.m[1][0]) / s;
            q.z = (m.m[0][2] + m.m[2][0]) / s;
        }
        else if (m.m[1][1] > m.m[2][2])
        {
            double s = sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0; // s = 4*qy
            q.w = (m.m[0][2] - m.m[2][0]) / s;
            q.x = (m.m[0][1] + m.m[1][0]) / s;
            q.y = 0.25 * s;
            q.z = (m.m[1][2] + m.m[2][1]) / s;
        }
        else
        {
            double s = sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0; // s = 4*qz
            q.w = (m.m[1][0] - m.m[0][1]) / s;
            q.x = (m.m[0][2] + m.m[2][0]) / s;
            q.y = (m.m[1][2] + m.m[2][1]) / s;
            q.z = 0.25 * s;
        }
        return ImuQuaternion::normalize(q);
    }

    Quaternion rotateDmpQuaternion(const Quaternion &dmpQuat, MountOrientation orientation)
    {
        // Derivation: DMP defines v_world = dmpQuat * v_sensor * conj(dmpQuat).
        // matrixFor defines v_boat = M * v_sensor, i.e. (in quaternion form)
        // v_boat = q_M * v_sensor * conj(q_M), so v_sensor = conj(q_M) *
        // v_boat * q_M. Substituting: v_world = (dmpQuat * conj(q_M)) *
        // v_boat * conj(dmpQuat * conj(q_M)) - so the boat-frame quaternion
        // is dmpQuat * conj(q_M). For Forward (q_M = identity) this
        // reduces to dmpQuat * identity = dmpQuat exactly, the required
        // no-op case.
        Quaternion qM = quaternionFor(orientation);
        return ImuQuaternion::multiply(dmpQuat, ImuQuaternion::conjugate(qM));
    }

    bool isValidRotation(const Mat3 &m, double tolerance)
    {
        if (fabs(m.determinant() - 1.0) > tolerance)
            return false;
        Mat3 prod = m.multiply(m.transposed());
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                double expected = (i == j) ? 1.0 : 0.0;
                if (fabs(prod.m[i][j] - expected) > tolerance)
                    return false;
            }
        return true;
    }
}
