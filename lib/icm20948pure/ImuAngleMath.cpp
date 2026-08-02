#include "ImuAngleMath.h"
#include <math.h>

namespace ImuAngleMath
{
    double wrap360(double deg)
    {
        deg = fmod(deg, 360.0);
        if (deg < 0)
            deg += 360.0;
        return deg;
    }

    double wrap180(double deg)
    {
        deg = fmod(deg, 360.0);
        if (deg <= -180.0)
            deg += 360.0;
        else if (deg > 180.0)
            deg -= 360.0;
        return deg;
    }

    double shortestDiff(double fromDeg, double toDeg)
    {
        return wrap180(toDeg - fromDeg);
    }

    double circularInterpolate(double aDeg, double bDeg, double t)
    {
        double diff = shortestDiff(aDeg, bDeg);
        return wrap360(aDeg + diff * t);
    }

    double circularWeightedAverage(const double *anglesDeg, const double *weights, int n)
    {
        if (n <= 0)
            return 0.0;
        double sx = 0, sy = 0;
        for (int i = 0; i < n; i++)
        {
            double rad = anglesDeg[i] * (M_PI / 180.0);
            sx += weights[i] * cos(rad);
            sy += weights[i] * sin(rad);
        }
        if (sx == 0.0 && sy == 0.0)
            return 0.0;
        return wrap360(atan2(sy, sx) * (180.0 / M_PI));
    }

    double circularLowPass(double prevDeg, double newDeg, double alpha)
    {
        double diff = shortestDiff(prevDeg, newDeg);
        return wrap360(prevDeg + diff * alpha);
    }

    void UnwrappedAccumulator::reset(double deg)
    {
        lastDeg = wrap360(deg);
        unwrapped = lastDeg;
        initialized = true;
    }

    double UnwrappedAccumulator::update(double deg)
    {
        deg = wrap360(deg);
        if (!initialized)
        {
            reset(deg);
            return unwrapped;
        }
        double diff = shortestDiff(lastDeg, deg);
        unwrapped += diff;
        lastDeg = deg;
        return unwrapped;
    }
}
