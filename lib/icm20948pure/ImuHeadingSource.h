#pragma once
#include "ImuTypes.h"

/*
  Selects and blends the active heading source between the software 9-axis
  fusion candidate and the tilt-compensated compass.
*/

struct SourceCandidate
{
    bool valid = false;
    double headingDeg = 0;
    HeadingQuality quality = HeadingQuality::Invalid;

    // Explicit constructors, not just relying on aggregate-init brace
    // lists - a default member initializer makes this a non-aggregate
    // under some toolchains' C++ standard defaults (confirmed the hard
    // way with Mat3 in ImuCoordinateTransform: identical brace-init code
    // compiled fine under the native test env but failed on the ESP32
    // toolchain with "could not convert... to const SourceCandidate").
    SourceCandidate() {}
    SourceCandidate(bool v, double h, HeadingQuality q) : valid(v), headingDeg(h), quality(q) {}
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
                   const SourceCandidate &fusion, const SourceCandidate &compass,
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

    static HeadingSource pickAuto(const SourceCandidate &fusion, const SourceCandidate &compass);
    static const SourceCandidate &candidateFor(HeadingSource s, const SourceCandidate &fusion, const SourceCandidate &compass);
};
