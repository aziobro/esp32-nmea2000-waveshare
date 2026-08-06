#include "ImuQuaternion.h"
#include "ImuAngleMath.h"
#include <math.h>

namespace ImuQuaternion
{
    double normSquared(const Quaternion &q)
    {
        return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    }

    double norm(const Quaternion &q)
    {
        return sqrt(normSquared(q));
    }

    Quaternion normalize(const Quaternion &q)
    {
        double n = norm(q);
        if (!(n > 1e-9)) // also catches NaN (comparisons with NaN are false)
            return Quaternion();
        Quaternion r;
        r.w = q.w / n;
        r.x = q.x / n;
        r.y = q.y / n;
        r.z = q.z / n;
        return r;
    }

    bool isFinite(const Quaternion &q)
    {
        return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
    }

    bool isValidUnit(const Quaternion &q, double tolerance)
    {
        if (!isFinite(q))
            return false;
        double n = norm(q);
        return fabs(n - 1.0) <= tolerance;
    }

    void toEuler(const Quaternion &q, double &rollRad, double &pitchRad, double &yawRad)
    {
        double w = q.w, x = q.x, y = q.y, z = q.z;
        rollRad = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        double sinp = 2.0 * (w * y - z * x);
        if (sinp > 1.0)
            sinp = 1.0;
        if (sinp < -1.0)
            sinp = -1.0;
        pitchRad = asin(sinp);
        yawRad = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    Quaternion fromEuler(double rollRad, double pitchRad, double yawRad)
    {
        double cy = cos(yawRad * 0.5), sy = sin(yawRad * 0.5);
        double cp = cos(pitchRad * 0.5), sp = sin(pitchRad * 0.5);
        double cr = cos(rollRad * 0.5), sr = sin(rollRad * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    Quaternion multiply(const Quaternion &a, const Quaternion &b)
    {
        Quaternion r;
        r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return r;
    }

    Quaternion conjugate(const Quaternion &q)
    {
        Quaternion r;
        r.w = q.w;
        r.x = -q.x;
        r.y = -q.y;
        r.z = -q.z;
        return r;
    }

    double yawDifferenceDeg(const Quaternion &a, const Quaternion &b)
    {
        double ar, ap, ay, br, bp, by;
        toEuler(a, ar, ap, ay);
        toEuler(b, br, bp, by);
        return ImuAngleMath::shortestDiff(ay * (180.0 / M_PI), by * (180.0 / M_PI));
    }
}
