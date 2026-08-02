#include "ImuGyroCal.h"
#include <math.h>

void GyroCalEngine::start()
{
    st = GyroCalState::Collecting;
    count = 0;
    mean = Vec3(0, 0, 0);
    m2 = Vec3(0, 0, 0);
}

void GyroCalEngine::cancel()
{
    st = GyroCalState::Idle;
    count = 0;
    mean = Vec3(0, 0, 0);
    m2 = Vec3(0, 0, 0);
}

bool GyroCalEngine::addSample(const Vec3 &gyroDegPerSec, double accelMagG, const GyroCalConfig &cfg)
{
    if (st != GyroCalState::Collecting)
        return false;

    if (gyroDegPerSec.norm() > cfg.maxGyroMagDegPerSec)
        return false;
    if (fabs(accelMagG - 1.0) > cfg.accelMagToleranceG)
        return false;

    // Welford's online mean/variance, one axis at a time.
    count++;
    double n = (double)count;
    Vec3 delta(gyroDegPerSec.x - mean.x, gyroDegPerSec.y - mean.y, gyroDegPerSec.z - mean.z);
    mean = mean + delta * (1.0 / n);
    Vec3 delta2(gyroDegPerSec.x - mean.x, gyroDegPerSec.y - mean.y, gyroDegPerSec.z - mean.z);
    m2 = Vec3(m2.x + delta.x * delta2.x, m2.y + delta.y * delta2.y, m2.z + delta.z * delta2.z);

    if (count >= cfg.requiredSamples)
    {
        Vec3 stddev = resultStdDevDegPerSec();
        if (stddev.x > cfg.maxStdDevDegPerSec || stddev.y > cfg.maxStdDevDegPerSec || stddev.z > cfg.maxStdDevDegPerSec)
            st = GyroCalState::Failed;
        else
            st = GyroCalState::Done;
    }
    return true;
}

Vec3 GyroCalEngine::resultStdDevDegPerSec() const
{
    if (count < 2)
        return Vec3(0, 0, 0);
    double n = (double)count;
    return Vec3(sqrt(m2.x / n), sqrt(m2.y / n), sqrt(m2.z / n));
}

namespace ImuRateOfTurn
{
    double lowPass(double prevDegPerSec, double newDegPerSec, double alpha)
    {
        return prevDegPerSec + (newDegPerSec - prevDegPerSec) * alpha;
    }

    double derivedFromHeadingDegPerSec(double headingUnwrappedDegNow, double headingUnwrappedDegPrev, double dtSec)
    {
        if (dtSec <= 1e-6)
            return 0.0;
        return (headingUnwrappedDegNow - headingUnwrappedDegPrev) / dtSec;
    }

    bool disagreesWithHeadingDerivative(double gyroRotDegPerSec, double headingDerivedDegPerSec, double toleranceDegPerSec)
    {
        return fabs(gyroRotDegPerSec - headingDerivedDegPerSec) > toleranceDegPerSec;
    }
}
