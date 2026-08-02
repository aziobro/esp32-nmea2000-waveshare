#pragma once
#include "ImuTypes.h"

/*
  Tracks the calibrated magnetic field magnitude and flags disturbances
  (nearby ferrous/electrical interference) with hysteresis, so quality
  doesn't rapidly flip between good/bad on isolated noisy samples.
*/

struct MagMonitorConfig
{
    // All provisional - see doc/IcmHeadingArchitecture.md; real limits
    // depend on the installation's own ambient field, not knowable
    // without hardware.
    double minMagnitude = 15.0;
    double maxMagnitude = 80.0;
    double maxSuddenChangePercent = 20.0;
    double referenceMagnitude = 0; // 0 = not yet established (e.g. from calibration)
    double maxDeviationFromReferencePercent = 25.0;
    double filterAlpha = 0.1;
    int hysteresisSamples = 5;
};

enum class MagDisturbanceState
{
    Unknown,
    Good,
    Disturbed
};

class MagMonitor
{
public:
    void reset();
    void update(const Vec3 &calibratedMag, const MagMonitorConfig &cfg = MagMonitorConfig());

    double instantaneousMagnitude() const { return instMag; }
    double filteredMagnitude() const { return filtMag; }
    double minMagnitudeSeen() const { return minMag; }
    double maxMagnitudeSeen() const { return maxMag; }
    MagDisturbanceState state() const { return st; }

private:
    bool initialized = false;
    double instMag = 0, filtMag = 0, prevMag = 0;
    double minMag = 1e18, maxMag = -1e18;
    int goodStreak = 0, badStreak = 0;
    MagDisturbanceState st = MagDisturbanceState::Unknown;
};
