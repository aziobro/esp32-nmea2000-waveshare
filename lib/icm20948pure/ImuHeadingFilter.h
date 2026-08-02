#pragma once
#include "ImuTypes.h"

/*
  Configurable circular heading filter applied AFTER source selection
  (ImuHeadingSource already handles the source-transition blend - this
  smooths ordinary sample-to-sample noise from whichever source is
  currently active). Always uses ImuAngleMath's circular operations -
  never plain arithmetic, which breaks near the 0/360 wrap.
*/

struct HeadingFilterConfig
{
    bool enabled = true;
    double timeConstantSec = 1.0;       // provisional - normal (steady-state) smoothing
    double fastTimeConstantSec = 0.2;   // provisional - used when turning quickly
    double rotThresholdForFastDegPerSec = 5.0; // provisional - |ROT| above this counts as "turning"
    double maxJumpDegPerSample = 30.0;  // provisional - beyond this, snap instead of smooth (assume a genuine fast turn or an already-handled source switch, not filter it away)
};

class HeadingFilter
{
public:
    void reset(double headingDeg);
    bool isInitialized() const { return initialized; }

    // rotDegPerSec is used only to pick the time constant (motion-adaptive
    // response) - the caller's own rate-of-turn estimate, not derived here.
    double update(double newHeadingDeg, double dtSec, double rotDegPerSec, const HeadingFilterConfig &cfg = HeadingFilterConfig());

    double value() const { return current; }

private:
    bool initialized = false;
    double current = 0;
};
