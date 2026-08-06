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

    // --- DMP euler. ---
    double dmpRollRad = 0, dmpPitchRad = 0, dmpYawRad = 0;
    if (in.haveDmpSample)
        ImuQuaternion::toEuler(in.dmpQuat, dmpRollRad, dmpPitchRad, dmpYawRad);
    out.dmpRollDeg = toDeg(dmpRollRad);
    out.dmpPitchDeg = toDeg(dmpPitchRad);

    // --- Roll/Pitch source: DMP if active, else plain accelerometer tilt
    // calc on the boat-frame accel vector. ---
    double rawRollRad, rawPitchRad;
    bool haveAttitudeSample;
    if (in.dmpOk)
    {
        rawRollRad = dmpRollRad;
        rawPitchRad = dmpPitchRad;
        haveAttitudeSample = in.haveDmpSample;
    }
    else
    {
        rawRollRad = atan2(in.accelBoat.y, in.accelBoat.z);
        rawPitchRad = atan2(-in.accelBoat.x, sqrt(in.accelBoat.y * in.accelBoat.y + in.accelBoat.z * in.accelBoat.z));
        haveAttitudeSample = true;
    }

    double rollDeg = 0, pitchDeg = 0;
    if (haveAttitudeSample)
    {
        double rawRollDeg = in.rollInvert ? -toDeg(rawRollRad) : toDeg(rawRollRad);
        double rawPitchDeg = in.pitchInvert ? -toDeg(rawPitchRad) : toDeg(rawPitchRad);
        rollDeg = ImuAngleMath::wrap180(rawRollDeg + in.rollOffsetDeg);
        pitchDeg = ImuAngleMath::wrap180(rawPitchDeg + in.pitchOffsetDeg);

        out.attitudeValid = true;
        out.rollDeg = rollDeg;
        out.pitchDeg = pitchDeg;
        out.rawRollDeg = rawRollDeg;
        out.rawPitchDeg = rawPitchDeg;
    }

    // --- Three candidate heading sources, all computed every cycle
    // regardless of mode - an independent software heading always runs,
    // even while DMP is active. ---
    double rawCompassHeadingDeg = in.magValid ? ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(magCal, toRad(rollDeg), toRad(pitchDeg))) : 0.0;

    auto fusionStart = std::chrono::steady_clock::now();
    fusion.update(Vec3(toRad(gyroCal.x), toRad(gyroCal.y), toRad(gyroCal.z)), in.accelBoat,
                  in.magValid ? magCal : Vec3(0, 0, 0), in.dtSec);
    out.fusionDurationUs = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - fusionStart).count();
    double fr, fp, fy;
    ImuQuaternion::toEuler(fusion.quaternion(), fr, fp, fy);
    double rawFusionHeadingDeg = ImuAngleMath::wrap360(toDeg(fy));
    bool fusionValid = in.magValid && (in.nowMs - in.taskStartMs) > 3000; // provisional settle time before trusting fusion's own convergence
    if (fusionValid)
    {
        double fusionCompassDiff = fabs(ImuAngleMath::shortestDiff(rawCompassHeadingDeg, rawFusionHeadingDeg));
        if (fusionCompassDiff > FUSION_COMPASS_MAX_DISAGREEMENT_DEG)
            fusionValid = false;
    }

    double rawDmpHeadingDeg = in.haveDmpSample ? ImuAngleMath::wrap360(toDeg(dmpYawRad)) : 0.0;

    DmpValidationConfig dmpCfg;
    uint32_t rejFlags = in.dmpOk ? dmpValidator.validate(in.dmpQuat, in.dmpFreshThisCycle, (double)in.dmpAgeMs,
                                                           rawCompassHeadingDeg, in.magValid,
                                                           (double)(in.nowMs - in.taskStartMs), in.dtSec, dmpCfg)
                                  : HR_SENSOR_READ_ERROR;
    if (!in.magValid)
        rejFlags |= HR_MAG_INVALID;
    if (magMonitor.state() == MagDisturbanceState::Disturbed)
        rejFlags |= HR_MAG_FIELD_CHANGE;
    if (in.magValid && (in.nowMs - in.taskStartMs) > 3000 &&
        fabs(ImuAngleMath::shortestDiff(rawCompassHeadingDeg, rawFusionHeadingDeg)) > FUSION_COMPASS_MAX_DISAGREEMENT_DEG)
        rejFlags |= HR_FUSION_COMPASS_DISAGREE;

    SourceCandidate dmpCandidate(in.dmpOk && in.haveDmpSample && rejFlags == HR_NONE, rawDmpHeadingDeg,
                                  (rejFlags == HR_NONE) ? HeadingQuality::Good : HeadingQuality::Invalid);
    SourceCandidate compassCandidate(in.magValid, rawCompassHeadingDeg,
                                      (magMonitor.state() == MagDisturbanceState::Disturbed) ? HeadingQuality::Poor : HeadingQuality::Good);
    SourceCandidate fusionCandidate(fusionValid, rawFusionHeadingDeg, HeadingQuality::Good);

    HeadingSourceSelector::Result selResult = sourceSelector.update(in.headingMode, fusionCandidate, dmpCandidate, compassCandidate,
                                                                      (uint32_t)in.nowMs, in.transitionMs);

    out.rawDmpHeadingDeg = rawDmpHeadingDeg;
    out.dmpCandidateValid = dmpCandidate.valid;
    out.rawCompassHeadingDeg = rawCompassHeadingDeg;
    out.rawFusionHeadingDeg = rawFusionHeadingDeg;
    out.fusionCandidateValid = fusionValid;
    out.headingSource = selResult.source;
    out.rejectionFlags = rejFlags;

    // Bridges a total loss of every heading source (selResult.valid ==
    // false) via bounded gyro dead reckoning rather than reporting "no
    // heading" the instant every candidate drops out - see
    // ImuHeadingHoldover.h. Source-agnostic: it doesn't know or care
    // WHY selResult was invalid (magnetic disturbance, DMP staleness,
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
