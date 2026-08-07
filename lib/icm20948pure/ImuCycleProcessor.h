#pragma once
#include "ImuTypes.h"
#include "ImuCalibration.h"
#include "ImuDeviationTable.h"
#include "ImuFusion.h"
#include "ImuHeadingSource.h"
#include "ImuMagMonitor.h"
#include "ImuHeadingFilter.h"
#include "ImuHeadingHoldover.h"
#include "ImuAngleMath.h"

/*
  The actual per-cycle heading/attitude pipeline: everything between "raw
  boat-frame samples" and "one fully-resolved ImuCycleOutput", extracted
  out of GwIcm20948Task.cpp so the SAME code drives both the real hardware
  task and debug replay (see GwIcm20948ReplayTask.h) - not a parallel
  reimplementation that could silently drift from what actually ships.

  Deliberately excludes anything hardware- or side-effect-related (I2C
  reads, N2K sends, web data, config reads, the "C" button calibration
  preview values, diagnostic CSV capture) - those stay in the task layer,
  which builds an ImuCycleInput each cycle, calls process(), and does its
  own thing with the ImuCycleOutput. That boundary is what makes this
  class pure C++ with no Arduino dependency, so it compiles and is
  unit-tested under the native test env exactly like every other
  lib/icm20948pure module.

  One instance's lifetime should match one task's lifetime (real or
  replay) - it owns the same persistent, cycle-to-cycle state the
  original inline code carried as function-local statics (fusion filter,
  source selector, mag disturbance monitor, heading filter, heading unwrap
  accumulator, ROT low-pass state).
*/

struct ImuCycleInput
{
    // Already boat-frame (post ImuCoordinateTransform) samples.
    Vec3 accelBoat;
    Vec3 gyroBoat;
    Vec3 magBoat;
    bool magValid = true;

    double dtSec = 0;
    unsigned long nowMs = 0;
    unsigned long taskStartMs = 0; // for fusion settle-time gating

    ImuCalibration cal;
    DeviationTable deviationTable;
    bool deviationEnabled = false;

    HeadingSourceMode headingMode = HeadingSourceMode::Auto;
    uint32_t transitionMs = 1000;

    bool rollInvert = false, pitchInvert = false;
    double rollOffsetDeg = 0, pitchOffsetDeg = 0;

    bool hdgInvert = false;
    double hdgOffsetDeg = 0;

    bool filterEnabled = false;
    double filterTimeConstantSec = 1.0;

    bool rotInvert = false;
    double rotFiltAlpha = 1.0;

    // Bridges brief total-loss-of-heading-source gaps via gyro dead
    // reckoning - see ImuHeadingHoldover.h. Defaults to that struct's own
    // defaults if the caller doesn't set it.
    HeadingHoldoverConfig headingHoldoverConfig;
};

struct ImuCycleOutput
{
    bool attitudeValid = false;
    double rollDeg = 0, pitchDeg = 0;
    double rawRollDeg = 0, rawPitchDeg = 0; // after invert, pre fine-offset

    bool headingValid = false;
    double headingDeg = 0;
    // selResult.headingDeg before invert/offset/deviation/filter - the
    // "C" button calibration preview needs this exact pre-correction
    // value, same as the original inline code.
    double preCorrectionHeadingDeg = 0;

    double rotDegPerSec = 0;
    double rotDerivedDegPerSec = 0;
    bool rotDisagrees = false;

    HeadingSource headingSource = HeadingSource::None;
    HeadingQuality headingQuality = HeadingQuality::Invalid;
    uint32_t rejectionFlags = HR_NONE;

    // True only while actively gyro-dead-reckoning through a total loss
    // of every heading source (see ImuHeadingHoldover.h) - the "clearly
    // identified gyro-only continuation" flag. Deliberately excluded
    // from PGN 127250 transmission by the task layer - see
    // GwIcm20948Task.cpp's send gate and doc/IcmHeadingValidityAudit.md.
    bool headingHoldover = false;
    HeadingHoldoverState headingHoldoverState = HeadingHoldoverState::Lost;

    Vec3 magCorrected;
    double magMagnitude = 0;
    MagDisturbanceState magDisturbanceState = MagDisturbanceState::Unknown;

    double rawCompassHeadingDeg = 0;
    double rawFusionHeadingDeg = 0;
    bool fusionCandidateValid = false;

    // Time spent in just the fusion.update() call, for the Performance
    // panel (see doc/IcmPerformanceReview.md) - measured with
    // std::chrono rather than an injected clock callback, since chrono
    // is portable pure C++ (no Arduino dependency) and the overhead of
    // two now() calls is negligible next to what it measures.
    double fusionDurationUs = 0;
};

class ImuCycleProcessor
{
public:
    ImuCycleOutput process(const ImuCycleInput &in);

private:
    MahonyFusion fusion;
    HeadingSourceSelector sourceSelector;
    MagMonitor magMonitor;
    HeadingFilter headingFilter;
    ImuHeadingHoldover headingHoldover;
    ImuAngleMath::UnwrappedAccumulator headingAccumulator;

    double filteredRotDegPerSec = 0;
    bool haveFilteredRot = false;
};
