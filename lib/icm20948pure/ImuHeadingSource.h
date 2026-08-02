#pragma once
#include "ImuTypes.h"

/*
  Validates the DMP's quaternion sample and selects/blends the active
  heading source. This is the module that directly implements the
  project's primary design goal: DMP yaw is treated as one candidate
  source among several (software compass, software fusion), validated
  before use, not trusted unconditionally - see doc/IcmHeadingArchitecture.md.
*/

// --- DMP sample validation ---

struct DmpValidationConfig
{
    double maxSampleAgeMs = 500;           // provisional - tune once real FIFO timing is known
    double quaternionNormTolerance = 0.05; // provisional
    int minConsecutiveValidSamples = 3;    // provisional
    double startupConvergenceMs = 3000;    // provisional - DMP needs time to converge after init
    double maxDisagreementWithCompassDeg = 30; // provisional
    double maxJumpDegPerSec = 120;             // provisional - implausibly fast heading change
};

class DmpValidator
{
public:
    void reset();

    // Call once per cycle with the latest DMP sample (or the previous
    // one carried forward, with sampleFresh=false, if the FIFO produced
    // nothing new this cycle). Returns a HeadingRejectReason bitmask (0 =
    // HR_NONE = valid). compassHeadingDeg/compassValid let the disagreement
    // check run even when compass isn't the active output source.
    uint32_t validate(const Quaternion &q, bool sampleFresh, double ageMs,
                       double compassHeadingDeg, bool compassValid,
                       double elapsedSinceInitMs, double dtSec,
                       const DmpValidationConfig &cfg = DmpValidationConfig());

private:
    int consecutiveValid = 0;
    bool havePrevHeading = false;
    double prevHeadingDeg = 0;
};

// --- Source selection / blending ---

struct SourceCandidate
{
    bool valid = false;
    double headingDeg = 0;
    HeadingQuality quality = HeadingQuality::Invalid;
};

struct SourceTransition
{
    HeadingSource from = HeadingSource::None;
    HeadingSource to = HeadingSource::None;
    uint32_t timestampMs = 0;
    double headingDiffDeg = 0;
};

class HeadingSourceSelector
{
public:
    struct Result
    {
        bool valid = false;
        double headingDeg = 0;
        HeadingSource source = HeadingSource::None;
        HeadingQuality quality = HeadingQuality::Invalid;
    };

    // mode DiagnosticOnly behaves identically to Auto for source
    // selection/blending purposes - it's the caller's job to not
    // transmit PGN 127250 in that mode, this module just reports the
    // best current estimate either way (useful for the web UI even when
    // transmission is off).
    Result update(HeadingSourceMode mode,
                   const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass,
                   uint32_t nowMs, uint32_t transitionDurationMs = 1000);

    bool hasRecentTransition() const { return hasTransition; }
    SourceTransition lastTransition() const { return transition; }

private:
    HeadingSource activeSource = HeadingSource::None;
    bool blending = false;
    uint32_t blendStartMs = 0;
    uint32_t blendDurationMs = 1000;
    double blendFromHeadingDeg = 0;
    double blendToHeadingDeg = 0;
    double lastOutputHeadingDeg = 0;

    bool hasTransition = false;
    SourceTransition transition;

    static HeadingSource pickAuto(const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass);
    static const SourceCandidate &candidateFor(HeadingSource s, const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass);
};
