#include "ImuCycleProcessor.h"
#include "ImuQuaternion.h"
#include "ImuCompass.h"
#include "ImuGyroCal.h" // ImuRateOfTurn::lowPass/derivedFromHeadingDegPerSec/disagreesWithHeadingDerivative
#include <math.h>

namespace
{
    double toDeg(double rad) { return rad * (180.0 / M_PI); }
    double toRad(double deg) { return deg * (M_PI / 180.0); }
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
    magMonitor.update(magCal, magCfg);
    out.magCorrected = magCal;
    out.magMagnitude = magMagnitude;
    out.magDisturbanceState = magMonitor.state();

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
    double rawCompassHeadingDeg = ImuAngleMath::wrap360(ImuCompass::rawHeadingDeg(magCal, toRad(rollDeg), toRad(pitchDeg)));

    fusion.update(Vec3(toRad(gyroCal.x), toRad(gyroCal.y), toRad(gyroCal.z)), in.accelBoat, magCal, in.dtSec);
    double fr, fp, fy;
    ImuQuaternion::toEuler(fusion.quaternion(), fr, fp, fy);
    double rawFusionHeadingDeg = ImuAngleMath::wrap360(toDeg(fy));
    bool fusionValid = (in.nowMs - in.taskStartMs) > 3000; // provisional settle time before trusting fusion's own convergence

    double rawDmpHeadingDeg = in.haveDmpSample ? ImuAngleMath::wrap360(toDeg(dmpYawRad)) : 0.0;

    DmpValidationConfig dmpCfg;
    uint32_t rejFlags = in.dmpOk ? dmpValidator.validate(in.dmpQuat, in.dmpFreshThisCycle, (double)in.dmpAgeMs,
                                                           rawCompassHeadingDeg, true,
                                                           (double)(in.nowMs - in.taskStartMs), in.dtSec, dmpCfg)
                                  : HR_SENSOR_READ_ERROR;
    if (magMonitor.state() == MagDisturbanceState::Disturbed)
        rejFlags |= HR_MAG_FIELD_CHANGE;

    SourceCandidate dmpCandidate(in.dmpOk && in.haveDmpSample && rejFlags == HR_NONE, rawDmpHeadingDeg,
                                  (rejFlags == HR_NONE) ? HeadingQuality::Good : HeadingQuality::Invalid);
    SourceCandidate compassCandidate(true, rawCompassHeadingDeg,
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
    out.headingQuality = selResult.quality;
    out.rejectionFlags = rejFlags;
    out.preCorrectionHeadingDeg = selResult.headingDeg;

    bool haveHeadingSample = selResult.valid;
    if (haveHeadingSample)
    {
        double corrected = in.hdgInvert ? -selResult.headingDeg : selResult.headingDeg;
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
