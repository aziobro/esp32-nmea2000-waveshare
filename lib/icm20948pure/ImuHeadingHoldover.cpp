#include "ImuHeadingHoldover.h"
#include "ImuAngleMath.h"

ImuHeadingHoldover::Result ImuHeadingHoldover::update(bool sourceValid, double sourceHeadingDeg, HeadingQuality sourceQuality,
                                                        double rotDegPerSec, double dtSec, uint32_t nowMs,
                                                        const HeadingHoldoverConfig &cfg)
{
    Result result;

    if (!sourceValid)
    {
        consecutiveValid = 0;
        recovering = false;

        if (state == HeadingHoldoverState::Tracking)
        {
            state = HeadingHoldoverState::Holdover;
            holdoverStartMs = nowMs;
        }

        if (state == HeadingHoldoverState::Holdover)
        {
            uint32_t elapsed = nowMs - holdoverStartMs;
            if (elapsed > (uint32_t)cfg.maxHoldoverMs)
            {
                state = HeadingHoldoverState::Lost;
            }
            else
            {
                heldHeadingDeg = ImuAngleMath::wrap360(heldHeadingDeg + rotDegPerSec * dtSec);
                result.valid = true;
                result.headingDeg = heldHeadingDeg;
                result.quality = HeadingQuality::Poor;
                result.state = HeadingHoldoverState::Holdover;
                result.gyroOnly = true;
                result.holdoverElapsedMs = elapsed;
                return result;
            }
        }

        result.valid = false;
        result.state = HeadingHoldoverState::Lost;
        return result;
    }

    // Source valid this cycle.
    consecutiveValid++;

    if (state == HeadingHoldoverState::Tracking)
    {
        heldHeadingDeg = sourceHeadingDeg;
        result.valid = true;
        result.headingDeg = sourceHeadingDeg;
        result.quality = sourceQuality;
        result.state = HeadingHoldoverState::Tracking;
        return result;
    }

    if (!everTracked)
    {
        // The very first fix ever - snap directly, nothing to recover
        // from (same precedent HeadingSourceSelector uses for "first
        // valid source after having none"). The consecutive-samples
        // requirement below is about RECOVERING from an actual loss, not
        // gating initial acquisition.
        state = HeadingHoldoverState::Tracking;
        everTracked = true;
        heldHeadingDeg = sourceHeadingDeg;
        result.valid = true;
        result.headingDeg = sourceHeadingDeg;
        result.quality = sourceQuality;
        result.state = HeadingHoldoverState::Tracking;
        return result;
    }

    // Recovering from Holdover or Lost - require N consecutive good
    // samples before trusting it again (a single flickering good sample
    // shouldn't immediately snap back).
    if (consecutiveValid < cfg.minConsecutiveSamplesToRecover)
    {
        if (state == HeadingHoldoverState::Holdover)
        {
            heldHeadingDeg = ImuAngleMath::wrap360(heldHeadingDeg + rotDegPerSec * dtSec);
            result.valid = true;
            result.headingDeg = heldHeadingDeg;
            result.quality = HeadingQuality::Poor;
            result.state = HeadingHoldoverState::Holdover;
            result.gyroOnly = true;
            return result;
        }
        // Lost - no continuity worth dead-reckoning from; stay silent
        // while counting toward recovery.
        result.valid = false;
        result.state = HeadingHoldoverState::Lost;
        return result;
    }

    // Threshold met - recover. From Holdover, blend circularly from the
    // held (dead-reckoned) heading to the recovered source, so the
    // transition doesn't jump (and correctly handles crossing north via
    // ImuAngleMath::circularInterpolate). From Lost, snap directly -
    // there's no continuous position worth preserving after being fully
    // lost, same reasoning HeadingSourceSelector already uses for "first
    // valid source after having none".
    if (!recovering)
    {
        recovering = true;
        recoverBlendStartMs = nowMs;
        recoverBlendFromDeg = heldHeadingDeg;
        recoverFromHoldover = (state == HeadingHoldoverState::Holdover);
    }

    if (!recoverFromHoldover || cfg.recoveryBlendMs <= 0)
    {
        state = HeadingHoldoverState::Tracking;
        recovering = false;
        heldHeadingDeg = sourceHeadingDeg;
        result.valid = true;
        result.headingDeg = sourceHeadingDeg;
        result.quality = sourceQuality;
        result.state = HeadingHoldoverState::Tracking;
        return result;
    }

    uint32_t elapsed = nowMs - recoverBlendStartMs;
    if (elapsed >= (uint32_t)cfg.recoveryBlendMs)
    {
        state = HeadingHoldoverState::Tracking;
        recovering = false;
        heldHeadingDeg = sourceHeadingDeg;
        result.valid = true;
        result.headingDeg = sourceHeadingDeg;
        result.quality = sourceQuality;
        result.state = HeadingHoldoverState::Tracking;
        return result;
    }

    double t = (double)elapsed / (double)cfg.recoveryBlendMs;
    heldHeadingDeg = ImuAngleMath::circularInterpolate(recoverBlendFromDeg, sourceHeadingDeg, t);
    result.valid = true;
    result.headingDeg = heldHeadingDeg;
    result.quality = HeadingQuality::Poor;
    result.state = HeadingHoldoverState::Holdover;
    result.gyroOnly = false; // recovering via blend to a real source, not pure dead-reckoning anymore
    return result;
}
