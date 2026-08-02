#pragma once
#include "ImuTypes.h"

/*
  Bridges brief losses of every heading source (e.g. a magnetic
  disturbance knocking out the compass while DMP/fusion are also
  momentarily unavailable) by dead-reckoning via the gyro's rate of turn
  for a bounded time, rather than reporting "no heading" the instant
  every candidate source drops out. Sits between HeadingSourceSelector
  and the rest of ImuCycleProcessor's heading-finalization stage.

  Deliberately source-agnostic about WHY a source became invalid -
  magnetic disturbance, DMP staleness, a bad quaternion, anything else -
  holdover only ever looks at the selector's own valid/invalid boolean,
  so a magnetic disturbance can't "poison" gyro-only continuation via
  some special-case rejection; it triggers holdover exactly like any
  other loss-of-source would.

  A holdover heading is intentionally excluded from PGN 127250
  transmission (see ImuCycleOutput::headingHoldover, and
  GwIcm20948Task.cpp's send gate) - it's a time-limited ESTIMATE, not a
  sensor reading, and transmitting an autopilot-facing PGN from an
  estimate that could be drifting from accumulated gyro bias is exactly
  the kind of risk this project has been deliberately conservative about
  throughout. It's still reported as valid (with Poor quality) for
  on-screen/logged diagnostics.
*/

enum class HeadingHoldoverState
{
    Tracking, // a source is currently valid - output mirrors it directly
    Holdover, // no source valid - gyro-integrated dead reckoning, within the time limit
    Lost,     // holdover time limit exceeded (or no fix has ever been recovered) - no output
};

struct HeadingHoldoverConfig
{
    double maxHoldoverMs = 5000;            // provisional - see doc/IcmHeadingArchitecture.md
    int minConsecutiveSamplesToRecover = 3; // matches DmpValidator's own convention
    double recoveryBlendMs = 1000;          // circular blend duration when recovering FROM Holdover (not from Lost - see .cpp)
};

class ImuHeadingHoldover
{
public:
    struct Result
    {
        bool valid = false;
        double headingDeg = 0;
        HeadingQuality quality = HeadingQuality::Invalid;
        HeadingHoldoverState state = HeadingHoldoverState::Lost;
        // True only while actively gyro-dead-reckoning (Holdover, before
        // a recovering source's blend completes) - the "clearly
        // identified gyro-only continuation" flag callers should surface.
        bool gyroOnly = false;
        double holdoverElapsedMs = 0;
    };

    // sourceValid/sourceHeadingDeg/sourceQuality is HeadingSourceSelector's
    // own output for this cycle. rotDegPerSec is the current (already
    // filtered/inverted) rate of turn, used to integrate the held heading
    // forward while no source is valid.
    Result update(bool sourceValid, double sourceHeadingDeg, HeadingQuality sourceQuality,
                  double rotDegPerSec, double dtSec, uint32_t nowMs,
                  const HeadingHoldoverConfig &cfg = HeadingHoldoverConfig());

private:
    HeadingHoldoverState state = HeadingHoldoverState::Lost;
    double heldHeadingDeg = 0;
    uint32_t holdoverStartMs = 0;
    int consecutiveValid = 0;
    // True once we've reached Tracking at least once - distinguishes "the
    // very first fix ever" (snap immediately, nothing to recover from,
    // same precedent HeadingSourceSelector uses) from "recovering from an
    // actual loss" (requires minConsecutiveSamplesToRecover).
    bool everTracked = false;

    bool recovering = false;
    bool recoverFromHoldover = false;
    uint32_t recoverBlendStartMs = 0;
    double recoverBlendFromDeg = 0;
};
