#include "ImuHeadingSource.h"
#include "ImuQuaternion.h"
#include "ImuAngleMath.h"
#include <math.h>

void DmpValidator::reset()
{
    consecutiveValid = 0;
    havePrevHeading = false;
    prevHeadingDeg = 0;
}

uint32_t DmpValidator::validate(const Quaternion &q, bool sampleFresh, double ageMs,
                                 double compassHeadingDeg, bool compassValid,
                                 double elapsedSinceInitMs, double dtSec,
                                 const DmpValidationConfig &cfg)
{
    uint32_t flags = HR_NONE;

    if (elapsedSinceInitMs < cfg.startupConvergenceMs)
        flags |= HR_INITIALIZING;

    if (!sampleFresh && ageMs > cfg.maxSampleAgeMs)
        flags |= HR_DMP_STALE;

    if (!ImuQuaternion::isValidUnit(q, cfg.quaternionNormTolerance))
        flags |= HR_BAD_QUATERNION;

    double r, p, y;
    double headingDeg = 0;
    bool haveHeading = false;
    if ((flags & HR_BAD_QUATERNION) == 0)
    {
        ImuQuaternion::toEuler(q, r, p, y);
        headingDeg = ImuAngleMath::wrap360(y * (180.0 / M_PI));
        haveHeading = true;

        if (compassValid)
        {
            double diff = fabs(ImuAngleMath::shortestDiff(compassHeadingDeg, headingDeg));
            if (diff > cfg.maxDisagreementWithCompassDeg)
                flags |= HR_DMP_COMPASS_DISAGREE;
        }

        if (havePrevHeading && dtSec > 1e-6)
        {
            double rate = fabs(ImuAngleMath::shortestDiff(prevHeadingDeg, headingDeg)) / dtSec;
            if (rate > cfg.maxJumpDegPerSec)
                flags |= HR_SUDDEN_JUMP;
        }
    }

    if (flags == HR_NONE)
        consecutiveValid++;
    else
        consecutiveValid = 0;

    // Not enough consecutive good samples yet is treated the same as
    // still-initializing - both mean "not yet trustworthy", and the
    // reject-reason enum doesn't have a separate bit for it (see header).
    if (consecutiveValid < cfg.minConsecutiveValidSamples)
        flags |= HR_INITIALIZING;

    if (haveHeading)
    {
        prevHeadingDeg = headingDeg;
        havePrevHeading = true;
    }

    return flags;
}

// --- Source selection ---

HeadingSource HeadingSourceSelector::pickAuto(const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass)
{
    if (fusion.valid)
        return HeadingSource::SoftwareFusion;
    if (dmp.valid)
        return HeadingSource::Dmp;
    if (compass.valid)
        return HeadingSource::SoftwareCompass;
    return HeadingSource::None;
}

const SourceCandidate &HeadingSourceSelector::candidateFor(HeadingSource s, const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass)
{
    switch (s)
    {
    case HeadingSource::SoftwareFusion:
        return fusion;
    case HeadingSource::Dmp:
        return dmp;
    case HeadingSource::SoftwareCompass:
        return compass;
    default:
        return compass; // unreachable in practice (caller checks source != None first)
    }
}

HeadingSourceSelector::Result HeadingSourceSelector::update(
    HeadingSourceMode mode,
    const SourceCandidate &fusion, const SourceCandidate &dmp, const SourceCandidate &compass,
    uint32_t nowMs, uint32_t transitionDurationMs)
{
    Result result;

    HeadingSource desired;
    switch (mode)
    {
    case HeadingSourceMode::Dmp:
        desired = dmp.valid ? HeadingSource::Dmp : HeadingSource::None;
        break;
    case HeadingSourceMode::SoftwareCompass:
        desired = compass.valid ? HeadingSource::SoftwareCompass : HeadingSource::None;
        break;
    case HeadingSourceMode::SoftwareFusion:
        desired = fusion.valid ? HeadingSource::SoftwareFusion : HeadingSource::None;
        break;
    case HeadingSourceMode::DiagnosticOnly:
    case HeadingSourceMode::Auto:
    default:
        desired = pickAuto(fusion, dmp, compass);
        break;
    }

    if (desired == HeadingSource::None)
    {
        // Lost every source - report invalid, but remember we were
        // active so the NEXT valid source still gets a clean transition
        // record (rather than silently resuming with no history).
        if (activeSource != HeadingSource::None)
        {
            hasTransition = true;
            transition.from = activeSource;
            transition.to = HeadingSource::None;
            transition.timestampMs = nowMs;
            transition.headingDiffDeg = 0;
        }
        activeSource = HeadingSource::None;
        blending = false;
        result.valid = false;
        return result;
    }

    const SourceCandidate &desiredCandidate = candidateFor(desired, fusion, dmp, compass);

    if (activeSource == HeadingSource::None)
    {
        // First valid source after having none - snap directly, nothing
        // to blend from.
        activeSource = desired;
        lastOutputHeadingDeg = desiredCandidate.headingDeg;
        result.valid = true;
        result.headingDeg = desiredCandidate.headingDeg;
        result.source = desired;
        result.quality = desiredCandidate.quality;
        return result;
    }

    if (desired != activeSource && !blending)
    {
        // Source change - start a circular blend from the last output
        // toward the new source's current reading, so the displayed/
        // transmitted heading doesn't jump instantly.
        blending = true;
        blendStartMs = nowMs;
        blendDurationMs = transitionDurationMs;
        blendFromHeadingDeg = lastOutputHeadingDeg;
        blendToHeadingDeg = desiredCandidate.headingDeg;

        hasTransition = true;
        transition.from = activeSource;
        transition.to = desired;
        transition.timestampMs = nowMs;
        transition.headingDiffDeg = ImuAngleMath::shortestDiff(lastOutputHeadingDeg, desiredCandidate.headingDeg);

        activeSource = desired;
    }

    double outputHeading;
    if (blending)
    {
        // Keep tracking the (possibly still-moving) target while blending.
        blendToHeadingDeg = desiredCandidate.headingDeg;
        uint32_t elapsed = nowMs - blendStartMs;
        if (elapsed >= blendDurationMs || blendDurationMs == 0)
        {
            outputHeading = blendToHeadingDeg;
            blending = false;
        }
        else
        {
            double t = (double)elapsed / (double)blendDurationMs;
            outputHeading = ImuAngleMath::circularInterpolate(blendFromHeadingDeg, blendToHeadingDeg, t);
        }
    }
    else
    {
        outputHeading = desiredCandidate.headingDeg;
    }

    lastOutputHeadingDeg = outputHeading;
    result.valid = true;
    result.headingDeg = ImuAngleMath::wrap360(outputHeading);
    result.source = activeSource;
    result.quality = desiredCandidate.quality;
    return result;
}
