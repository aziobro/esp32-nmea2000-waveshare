#pragma once
#include "ImuTypes.h"
#include "ImuCalibration.h"
#include "ImuDeviationTable.h"
#include "ImuFusion.h"
#include "ImuHeadingSource.h"
#include "ImuMagMonitor.h"
#include "ImuHeadingFilter.h"
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
  DMP validator, source selector, mag disturbance monitor, heading
  filter, heading unwrap accumulator, ROT low-pass state).
*/

struct ImuCycleInput
{
    // Already boat-frame (post ImuCoordinateTransform) samples.
    Vec3 accelBoat;
    Vec3 gyroBoat;
    Vec3 magBoat;

    bool dmpOk = false;             // DMP hardware initialized/enabled at all
    bool haveDmpSample = false;     // a DMP quaternion has been read at least once since startup
    bool dmpFreshThisCycle = false; // a NEW DMP quaternion was read THIS cycle
    Quaternion dmpQuat;             // last known DMP quaternion - meaningful iff haveDmpSample
    unsigned long dmpAgeMs = 0;     // ms since the last fresh DMP sample

    double dtSec = 0;
    unsigned long nowMs = 0;
    unsigned long taskStartMs = 0; // for fusion settle-time and DMP validator uptime gating

    ImuCalibration cal;
    DeviationTable deviationTable;
    bool deviationEnabled = false;

    HeadingSourceMode headingMode = HeadingSourceMode::Dmp;
    uint32_t transitionMs = 1000;

    bool rollInvert = false, pitchInvert = false;
    double rollOffsetDeg = 0, pitchOffsetDeg = 0;

    bool hdgInvert = false;
    double hdgOffsetDeg = 0;

    bool filterEnabled = false;
    double filterTimeConstantSec = 1.0;

    bool rotInvert = false;
    double rotFiltAlpha = 1.0;
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

    Vec3 magCorrected;
    double magMagnitude = 0;
    MagDisturbanceState magDisturbanceState = MagDisturbanceState::Unknown;

    double dmpRollDeg = 0, dmpPitchDeg = 0;
    double rawDmpHeadingDeg = 0;
    bool dmpCandidateValid = false;
    double rawCompassHeadingDeg = 0;
    double rawFusionHeadingDeg = 0;
    bool fusionCandidateValid = false;
};

class ImuCycleProcessor
{
public:
    ImuCycleOutput process(const ImuCycleInput &in);

private:
    MahonyFusion fusion;
    DmpValidator dmpValidator;
    HeadingSourceSelector sourceSelector;
    MagMonitor magMonitor;
    HeadingFilter headingFilter;
    ImuAngleMath::UnwrappedAccumulator headingAccumulator;

    double filteredRotDegPerSec = 0;
    bool haveFilteredRot = false;
};
