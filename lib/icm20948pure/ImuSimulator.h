#pragma once
#include "ImuTypes.h"

/*
  Host-side synthetic sensor generator - lets the rest of the pipeline
  (compass, fusion, source selection) be exercised against
  mathematically-consistent fake data, both in unit tests and in the
  on-device debug replay mode (compile-time gated, see
  GwIcm20948Task.cpp's ICM_DEBUG_REPLAY).

  Generates readings already in BOAT FRAME (X=forward, Y=starboard,
  Z=down) at MountOrientation::Forward, i.e. this models "what the
  hardware adapter would hand the rest of the pipeline after coordinate
  transform" - it does not model raw per-chip-axis readings, since the
  coordinate transform itself is already exhaustively tested separately
  (ImuCoordinateTransform).

  Known simplification: the simulated gyroscope only reflects yaw rate
  (state.rateOfTurnDegPerSec) plus configured bias/noise - it does not
  derive X/Y gyro readings from roll/pitch changing over time between
  calls. Scenarios that vary roll/pitch (e.g. scenarioFixedHeadingChangingAttitude)
  still produce correct instantaneous accel/mag-derived roll/pitch/heading
  at each sampled time; only the gyro's own roll-rate/pitch-rate sensing
  isn't modeled. Documented rather than silently wrong.
*/

struct SimulatorState
{
    double headingDeg = 0;
    double rollDeg = 0;
    double pitchDeg = 0;
    double rateOfTurnDegPerSec = 0;

    double gyroBiasDegPerSec[3] = {0, 0, 0};
    double gyroNoiseDegPerSec = 0; // stddev-ish amplitude, 0 = deterministic

    // Uncalibrated sensor distortion the pipeline's own calibration is
    // supposed to remove: rawMag = cleanMag * softIronScale + hardIronBias
    // (elementwise scale, then offset).
    double magHardIronBias[3] = {0, 0, 0};
    double magSoftIronScale[3] = {1, 1, 1};
    double magNoise = 0;
    bool magDropout = false; // simulates a disconnected/failed magnetometer: raw reads all-zero

    double horizontalFieldMagnitude = 22.0; // arbitrary units, consistent within a scenario is all that matters
    double verticalFieldMagnitude = 45.0;   // northern-hemisphere-like dip
};

struct SimulatedSample
{
    Vec3 accelG;        // boat frame, g units, gravity only (no linear accel modeled)
    Vec3 gyroDegPerSec;  // boat frame
    Vec3 magRaw;         // boat frame, uncalibrated (see magHardIronBias/magSoftIronScale)
};

namespace ImuSimulator
{
    // The general-purpose generator every scenario function below is
    // built from - also directly usable in ad-hoc tests.
    SimulatedSample generateSample(const SimulatorState &state);

    // The 12 scripted scenarios from the project spec - each returns the
    // SimulatorState at a given elapsed time within the scenario, so
    // callers (tests, replay mode) can sample it at whatever rate they
    // like. durationSec is each scenario's own natural length; callers
    // may sample beyond it (the motion just continues/repeats where that
    // makes sense, e.g. rotations wrap).
    namespace Scenarios
    {
        SimulatorState level360Rotation(double timeSec, double periodSec = 30.0);
        SimulatorState rotation360With20DegHeel(double timeSec, double periodSec = 30.0);
        SimulatorState fixedHeadingChangingAttitude(double timeSec, double headingDeg = 90.0);
        SimulatorState hardIronOffset(double timeSec, double headingDeg = 45.0);
        SimulatorState ellipticalSoftIronDistortion(double timeSec, double headingDeg = 45.0);
        SimulatorState suddenMagneticDisturbance(double timeSec, double disturbanceStartSec = 5.0);
        SimulatorState slowGyroDrift(double timeSec, double driftDegPerSecPerSec = 0.001);
        SimulatorState headingWrapThroughNorth(double timeSec, double periodSec = 20.0);
        SimulatorState magnetometerDropout(double timeSec, double dropoutStartSec = 5.0);
    }
}
