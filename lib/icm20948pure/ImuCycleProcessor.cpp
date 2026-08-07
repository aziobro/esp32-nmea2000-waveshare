#include "ImuCycleProcessor.h"
#include "ImuQuaternion.h"
#include "ImuCompass.h"
#include "ImuGyroCal.h" // ImuRateOfTurn::lowPass/derivedFromHeadingDegPerSec/disagreesWithHeadingDerivative
#include <chrono>
#include <math.h>

namespace
{
    double toDeg(double rad) { return rad * (180.0 / M_PI); }
    double toRad(double deg) { return deg * (M_PI / 180.0); }
    constexpr double FUSION_COMPASS_MAX_DISAGREEMENT_DEG = 30.0;
}

ImuCycleOutput ImuCycleProcessor::process(const ImuCycleInput &in)
{
    ImuCycleOutput out;

    Vec3 gyroCal = ImuCalibrationOps::applyGyro(in.gyroBoat, in.cal);

    // --- Rate of Turn: low-pass filtered (alpha=1.0 = no smoothing). ---
    double rawRotDegPerSec = gyroCal.z;
    if (!haveFilteredRot)
    {
        filteredRotDegPerSec = rawRotDegPerSec;
        haveFilteredRot = true;
    }
    else
    {
        filteredRotDegPerSec = ImuRateOfTurn::lowPass(filteredRotDegPerSec, rawRotDegPerSec, in.rotFiltAlpha);
    }
    double rotDegPerSec = filteredRotDegPerSec;
    if (in.rotInvert)
        rotDegPerSec = -rotDegPerSec;
    out.rotDegPerSec = rotDegPerSec;

    // --- Magnetometer calibration. ---
    Vec3 magCal = ImuCalibrationOps::applyMag(in.magBoat, in.cal);
    double magMagnitude = magCal.norm();
    MagMonitorConfig magCfg;
    if (in.magValid)
        magMonitor.update(magCal, magCfg);
    out.magCorrected = magCal;
    out.magMagnitude = magMagnitude;
    out.magDisturbanceState = in.magValid ? magMonitor.state() : MagDisturbanceState::Disturbed;

    // --- Roll/Pitch: plain accelerometer tilt calc on the boat-frame accel vector. ---
    double rawRollRad = atan2(in.accelBoat.y, in.accelBoat.z);
    double rawPitchRad = atan2(-in.accelBoat.x, sqrt(in.accelBoat.y * in.accelBoat.y + in.accelBoat.z * in.accelBoat.z));

    double rollDeg = 0, pitchDeg = 0;
    double rawRollDeg = in.rollInvert ? -toDeg(rawRollRad) : toDeg(rawRollRad);
    double rawPitchDeg = in.pitchInvert ? -toDeg(rawPitchRad) : toDeg(rawPitchRad);
    rollDeg = ImuAngleMath::wrap180(rawRollDeg + in.rollOffsetDeg);
    pitchDeg = ImuAngleMath::wrap180(rawPitchDeg + in.pitchOffsetDeg);

    out.attitudeValid = true;
    out.rollDeg = rollDeg;
    out.pitchDeg = pitchDeg;
    out.rawRollDeg = rawRollDeg;
    out.rawPitchDeg = rawPitchDeg;

    // --- Software heading candidates, computed every cycle regardless of mode. ---
    double rawCompassHeadingDeg = in.magValid ? ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(magCal, toRad(rollDeg), toRad(pitchDeg))) : 0.0;

    auto fusionStart = std::chrono::steady_clock::now();
    fusion.update(Vec3(toRad(gyroCal.x), toRad(gyroCal.y), toRad(gyroCal.z)), in.accelBoat,
                  in.magValid ? magCal : Vec3(0, 0, 0), in.dtSec);
    out.fusionDurationUs = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - fusionStart).count();
    double fr, fp, fy;
    ImuQuaternion::toEuler(fusion.quaternion(), fr, fp, fy);
    double rawFusionHeadingDeg = ImuAngleMath::wrap360(-toDeg(fy));
    bool fusionValid = in.magValid && (in.nowMs - in.taskStartMs) > 3000; // provisional settle time before trusting fusion's own convergence
    if (fusionValid)
    {
        double fusionCompassDiff = fabs(ImuAngleMath::shortestDiff(rawCompassHeadingDeg, rawFusionHeadingDeg));
        if (fusionCompassDiff > FUSION_COMPASS_MAX_DISAGREEMENT_DEG)
            fusionValid = false;
    }

    uint32_t rejFlags = HR_NONE;
    if (!in.magValid)
        rejFlags |= HR_MAG_INVALID;
    if (magMonitor.state() == MagDisturbanceState::Disturbed)
        rejFlags |= HR_MAG_FIELD_CHANGE;
    if (in.magValid && (in.nowMs - in.taskStartMs) > 3000 &&
        fabs(ImuAngleMath::shortestDiff(rawCompassHeadingDeg, rawFusionHeadingDeg)) > FUSION_COMPASS_MAX_DISAGREEMENT_DEG)
        rejFlags |= HR_FUSION_COMPASS_DISAGREE;

    SourceCandidate compassCandidate(in.magValid, rawCompassHeadingDeg,
                                      (magMonitor.state() == MagDisturbanceState::Disturbed) ? HeadingQuality::Poor : HeadingQuality::Good);
    SourceCandidate fusionCandidate(fusionValid, rawFusionHeadingDeg, HeadingQuality::Good);

    HeadingSourceSelector::Result selResult = sourceSelector.update(in.headingMode, fusionCandidate, compassCandidate,
                                                                      (uint32_t)in.nowMs, in.transitionMs);

    out.rawCompassHeadingDeg = rawCompassHeadingDeg;
    out.rawFusionHeadingDeg = rawFusionHeadingDeg;
    out.fusionCandidateValid = fusionValid;
    out.headingSource = selResult.source;
    out.rejectionFlags = rejFlags;

    // Bridges a total loss of every heading source (selResult.valid ==
    // false) via bounded gyro dead reckoning rather than reporting "no
    // heading" the instant every candidate drops out - see
    // ImuHeadingHoldover.h. Source-agnostic: it doesn't know or care
    // WHY selResult was invalid (magnetic disturbance, invalid mag sample,
    // anything else), so it can't be "poisoned" by any particular cause.
    ImuHeadingHoldover::Result holdoverResult = headingHoldover.update(
        selResult.valid, selResult.headingDeg, selResult.quality, rotDegPerSec, in.dtSec, (uint32_t)in.nowMs, in.headingHoldoverConfig);

    out.headingQuality = holdoverResult.quality;
    out.headingHoldover = holdoverResult.gyroOnly;
    out.headingHoldoverState = holdoverResult.state;
    out.preCorrectionHeadingDeg = holdoverResult.headingDeg;

    bool haveHeadingSample = holdoverResult.valid;
    if (haveHeadingSample)
    {
        double corrected = in.hdgInvert ? -holdoverResult.headingDeg : holdoverResult.headingDeg;
        corrected = ImuAngleMath::wrap360(corrected + in.hdgOffsetDeg);
        if (in.deviationEnabled)
        {
            corrected = in.deviationTable.apply(corrected);
        }
        double headingDeg;
        if (in.filterEnabled)
        {
            HeadingFilterConfig fCfg;
            fCfg.timeConstantSec = in.filterTimeConstantSec;
            headingDeg = headingFilter.update(corrected, in.dtSec, rotDegPerSec, fCfg);
        }
        else
        {
            headingDeg = corrected;
        }

        out.headingValid = true;
        out.headingDeg = headingDeg;

        // Diagnostic-only ROT cross-check - never affects the transmitted
        // rate-of-turn value, only surfaced for diagnostics.
        double prevUnwrapped = headingAccumulator.isInitialized() ? headingAccumulator.value() : headingDeg;
        double newUnwrapped = headingAccumulator.update(headingDeg);
        double derivedRotDegPerSec = ImuRateOfTurn::derivedFromHeadingDegPerSec(newUnwrapped, prevUnwrapped, in.dtSec);
        out.rotDerivedDegPerSec = derivedRotDegPerSec;
        out.rotDisagrees = ImuRateOfTurn::disagreesWithHeadingDerivative(rotDegPerSec, derivedRotDegPerSec, 30.0);
    }
    else
    {
        out.headingValid = false;
    }

    return out;
}
