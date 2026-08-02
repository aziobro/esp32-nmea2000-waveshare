#include "ImuFusion.h"
#include "ImuQuaternion.h"
#include <math.h>

namespace
{
    // Rotation matrix from a unit quaternion (w,x,y,z), such that
    // v_reference = R(q) * v_boat - the standard quaternion-to-rotation-
    // matrix formula, matching the sense ImuQuaternion documents (q
    // rotates boat frame into the reference/level-north frame).
    struct RotMat
    {
        double r[3][3];
    };

    RotMat toRotMat(const Quaternion &q)
    {
        double w = q.w, x = q.x, y = q.y, z = q.z;
        RotMat m;
        m.r[0][0] = 1 - 2 * (y * y + z * z);
        m.r[0][1] = 2 * (x * y - w * z);
        m.r[0][2] = 2 * (x * z + w * y);
        m.r[1][0] = 2 * (x * y + w * z);
        m.r[1][1] = 1 - 2 * (x * x + z * z);
        m.r[1][2] = 2 * (y * z - w * x);
        m.r[2][0] = 2 * (x * z - w * y);
        m.r[2][1] = 2 * (y * z + w * x);
        m.r[2][2] = 1 - 2 * (x * x + y * y);
        return m;
    }

    Vec3 applyForward(const RotMat &m, const Vec3 &v)
    {
        return Vec3(
            m.r[0][0] * v.x + m.r[0][1] * v.y + m.r[0][2] * v.z,
            m.r[1][0] * v.x + m.r[1][1] * v.y + m.r[1][2] * v.z,
            m.r[2][0] * v.x + m.r[2][1] * v.y + m.r[2][2] * v.z);
    }

    // Applies R(q)^T (the inverse rotation, reference frame -> boat frame).
    Vec3 applyTranspose(const RotMat &m, const Vec3 &v)
    {
        return Vec3(
            m.r[0][0] * v.x + m.r[1][0] * v.y + m.r[2][0] * v.z,
            m.r[0][1] * v.x + m.r[1][1] * v.y + m.r[2][1] * v.z,
            m.r[0][2] * v.x + m.r[1][2] * v.y + m.r[2][2] * v.z);
    }

    Vec3 cross(const Vec3 &a, const Vec3 &b)
    {
        return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    Vec3 normalizeVec(const Vec3 &v, bool &ok)
    {
        double n = v.norm();
        if (!(n > 1e-9))
        {
            ok = false;
            return Vec3(0, 0, 0);
        }
        ok = true;
        return v * (1.0 / n);
    }
}

MahonyFusion::MahonyFusion(double kp_, double ki_) : kp(kp_), ki(ki_)
{
    reset();
}

void MahonyFusion::reset(const Quaternion &initial)
{
    q = ImuQuaternion::normalize(initial);
    integralFeedback = Vec3(0, 0, 0);
}

void MahonyFusion::applyCorrectedGyroStep(const Vec3 &gyroRadPerSec, const Vec3 &error, double dtSec)
{
    Vec3 corrected = gyroRadPerSec;
    if (kp > 0)
        corrected = corrected + error * kp;
    if (ki > 0)
    {
        integralFeedback = integralFeedback + error * (ki * dtSec);
        corrected = corrected + integralFeedback;
    }

    double gx = corrected.x, gy = corrected.y, gz = corrected.z;
    double w = q.w, x = q.x, y = q.y, z = q.z;

    // qDot = 0.5 * q (x) (0, gx, gy, gz), Hamilton product.
    double dw = -0.5 * (x * gx + y * gy + z * gz);
    double dx = 0.5 * (w * gx + y * gz - z * gy);
    double dy = 0.5 * (w * gy - x * gz + z * gx);
    double dz = 0.5 * (w * gz + x * gy - y * gx);

    Quaternion nq;
    nq.w = w + dw * dtSec;
    nq.x = x + dx * dtSec;
    nq.y = y + dy * dtSec;
    nq.z = z + dz * dtSec;
    q = ImuQuaternion::normalize(nq);
}

void MahonyFusion::updateImuOnly(const Vec3 &gyroRadPerSec, const Vec3 &accel, double dtSec)
{
    bool accelOk;
    Vec3 a = normalizeVec(accel, accelOk);
    Vec3 error(0, 0, 0);
    if (accelOk)
    {
        RotMat R = toRotMat(q);
        // Estimated gravity direction in boat frame: R(q)^T applied to
        // the fixed reference-frame gravity (0,0,1).
        Vec3 vEst = applyTranspose(R, Vec3(0, 0, 1));
        error = cross(a, vEst);
    }
    applyCorrectedGyroStep(gyroRadPerSec, error, dtSec);
}

void MahonyFusion::update(const Vec3 &gyroRadPerSec, const Vec3 &accel, const Vec3 &mag, double dtSec)
{
    bool magOk;
    Vec3 m = normalizeVec(mag, magOk);
    if (!magOk)
    {
        updateImuOnly(gyroRadPerSec, accel, dtSec);
        return;
    }

    bool accelOk;
    Vec3 a = normalizeVec(accel, accelOk);

    RotMat R = toRotMat(q);
    Vec3 error(0, 0, 0);

    if (accelOk)
    {
        Vec3 vEst = applyTranspose(R, Vec3(0, 0, 1));
        error = error + cross(a, vEst);
    }

    // Rotate the (normalized) measured mag reading into the reference
    // frame using the CURRENT attitude estimate, then collapse its
    // horizontal component to a pure-north magnitude (bx) - the standard
    // trick that lets yaw be corrected from mag without needing to know
    // magnetic declination/true heading a priori. The vertical/dip
    // component (bz) is trusted directly since roll/pitch are already
    // well-constrained by the accel correction above.
    Vec3 hRef = applyForward(R, m);
    double bx = sqrt(hRef.x * hRef.x + hRef.y * hRef.y);
    double bz = hRef.z;
    Vec3 wEst = applyTranspose(R, Vec3(bx, 0, bz));
    error = error + cross(m, wEst);

    applyCorrectedGyroStep(gyroRadPerSec, error, dtSec);
}
