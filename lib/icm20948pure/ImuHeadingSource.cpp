#include "ImuHeadingSource.h"
#include "ImuAngleMath.h"

// --- Source selection ---

HeadingSource HeadingSourceSelector::pickAuto(const SourceCandidate &fusion, const SourceCandidate &compass)
{
    if (fusion.valid)
        return HeadingSource::SoftwareFusion;
    if (compass.valid)
        return HeadingSource::SoftwareCompass;
    return HeadingSource::None;
}

const SourceCandidate &HeadingSourceSelector::candidateFor(HeadingSource s, const SourceCandidate &fusion, const SourceCandidate &compass)
{
    switch (s)
    {
    case HeadingSource::SoftwareFusion:
        return fusion;
    case HeadingSource::SoftwareCompass:
        return compass;
    default:
        return compass; // unreachable in practice (caller checks source != None first)
    }
}

HeadingSourceSelector::Result HeadingSourceSelector::update(
    HeadingSourceMode mode,
    const SourceCandidate &fusion, const SourceCandidate &compass,
    uint32_t nowMs, uint32_t transitionDurationMs)
{
    Result result;

    HeadingSource desired;
    switch (mode)
    {
    case HeadingSourceMode::SoftwareCompass:
        desired = compass.valid ? HeadingSource::SoftwareCompass : HeadingSource::None;
        break;
    case HeadingSourceMode::SoftwareFusion:
        desired = fusion.valid ? HeadingSource::SoftwareFusion : HeadingSource::None;
        break;
    case HeadingSourceMode::DiagnosticOnly:
    case HeadingSourceMode::Auto:
    default:
        desired = pickAuto(fusion, compass);
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

    const SourceCandidate &desiredCandidate = candidateFor(desired, fusion, compass);

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
