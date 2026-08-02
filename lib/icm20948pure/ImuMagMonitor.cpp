#include "ImuMagMonitor.h"
#include <math.h>

void MagMonitor::reset()
{
    initialized = false;
    instMag = filtMag = prevMag = 0;
    minMag = 1e18;
    maxMag = -1e18;
    goodStreak = badStreak = 0;
    st = MagDisturbanceState::Unknown;
}

void MagMonitor::update(const Vec3 &calibratedMag, const MagMonitorConfig &cfg)
{
    instMag = calibratedMag.norm();

    if (!initialized)
    {
        filtMag = instMag;
        prevMag = instMag;
        minMag = instMag;
        maxMag = instMag;
        initialized = true;
    }
    else
    {
        filtMag = filtMag + (instMag - filtMag) * cfg.filterAlpha;
        if (instMag < minMag)
            minMag = instMag;
        if (instMag > maxMag)
            maxMag = instMag;
    }

    bool bad = false;
    if (instMag < cfg.minMagnitude || instMag > cfg.maxMagnitude)
        bad = true;
    if (prevMag > 1e-9)
    {
        double changePercent = fabs(instMag - prevMag) / prevMag * 100.0;
        if (changePercent > cfg.maxSuddenChangePercent)
            bad = true;
    }
    if (cfg.referenceMagnitude > 1e-9)
    {
        double devPercent = fabs(instMag - cfg.referenceMagnitude) / cfg.referenceMagnitude * 100.0;
        if (devPercent > cfg.maxDeviationFromReferencePercent)
            bad = true;
    }

    if (bad)
    {
        badStreak++;
        goodStreak = 0;
    }
    else
    {
        goodStreak++;
        badStreak = 0;
    }

    if (badStreak >= cfg.hysteresisSamples)
        st = MagDisturbanceState::Disturbed;
    else if (goodStreak >= cfg.hysteresisSamples)
        st = MagDisturbanceState::Good;
    // else: keep previous state - hysteresis, avoids flip-flopping on
    // isolated samples.

    prevMag = instMag;
}
