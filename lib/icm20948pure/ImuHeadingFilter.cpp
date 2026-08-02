#include "ImuHeadingFilter.h"
#include "ImuAngleMath.h"
#include <math.h>

void HeadingFilter::reset(double headingDeg)
{
    current = ImuAngleMath::wrap360(headingDeg);
    initialized = true;
}

double HeadingFilter::update(double newHeadingDeg, double dtSec, double rotDegPerSec, const HeadingFilterConfig &cfg)
{
    newHeadingDeg = ImuAngleMath::wrap360(newHeadingDeg);

    if (!cfg.enabled || !initialized)
    {
        reset(newHeadingDeg);
        return current;
    }

    double diff = fabs(ImuAngleMath::shortestDiff(current, newHeadingDeg));
    if (diff > cfg.maxJumpDegPerSample)
    {
        // A jump this large is either a genuine fast turn or a source
        // switch already smoothed by ImuHeadingSource's own blend -
        // either way, filtering it further would just add lag to a real
        // change rather than removing noise.
        current = newHeadingDeg;
        return current;
    }

    double tau = (fabs(rotDegPerSec) > cfg.rotThresholdForFastDegPerSec) ? cfg.fastTimeConstantSec : cfg.timeConstantSec;
    double alpha = 1.0;
    if (tau > 1e-6)
    {
        alpha = 1.0 - exp(-dtSec / tau);
        if (alpha > 1.0)
            alpha = 1.0;
        if (alpha < 0.0)
            alpha = 0.0;
    }
    current = ImuAngleMath::circularLowPass(current, newHeadingDeg, alpha);
    return current;
}
