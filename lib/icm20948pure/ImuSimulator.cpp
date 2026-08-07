#include "ImuSimulator.h"
#include <math.h>
#include <stdlib.h>

namespace
{
    // Rotates a "level-frame" vector into what a boat tilted by
    // (rollRad,pitchRad) would read - same derivation as
    // test_compass.cpp's applyTilt (pitch first around Y by -pitch, then
    // roll around X by -roll), confirmed by the compass tilt-compensation
    // tests. Used here for both the gravity vector and the magnetic field
    // vector, since both are physically "a fixed external vector as seen
    // by a tilted sensor" - the identical situation.
    Vec3 tiltVector(const Vec3 &level, double rollRad, double pitchRad)
    {
        double cp = cos(pitchRad), sp = sin(pitchRad);
        double x1 = level.x * cp - level.z * sp;
        double y1 = level.y;
        double z1 = level.x * sp + level.z * cp;
        double cr = cos(rollRad), sr = sin(rollRad);
        double x2 = x1;
        double y2 = cr * y1 + sr * z1;
        double z2 = -sr * y1 + cr * z1;
        return Vec3(x2, y2, z2);
    }

    // Fixed horizontal+vertical field, expressed in a level boat at the
    // given true heading - same derivation as ImuCompass's test model.
    Vec3 levelFrameField(double headingDeg, double bh, double bv)
    {
        double h = headingDeg * (M_PI / 180.0);
        return Vec3(bh * cos(h), -bh * sin(h), bv);
    }

    // Simple deterministic pseudo-noise (not a real RNG - reproducible
    // across test runs without seeding concerns, which matters more here
    // than statistical quality). 0 amplitude means exactly 0 output.
    double pseudoNoise(double amplitude, double seed)
    {
        if (amplitude <= 0)
            return 0.0;
        double s = sin(seed * 12.9898) * 43758.5453;
        double frac = s - floor(s);
        return (frac * 2.0 - 1.0) * amplitude;
    }
}

namespace ImuSimulator
{
    SimulatedSample generateSample(const SimulatorState &state)
    {
        SimulatedSample s;
        double rollRad = state.rollDeg * (M_PI / 180.0);
        double pitchRad = state.pitchDeg * (M_PI / 180.0);

        s.accelG = tiltVector(Vec3(0, 0, 1.0), rollRad, pitchRad);

        s.gyroDegPerSec = Vec3(
            state.gyroBiasDegPerSec[0] + pseudoNoise(state.gyroNoiseDegPerSec, 1.1),
            state.gyroBiasDegPerSec[1] + pseudoNoise(state.gyroNoiseDegPerSec, 2.2),
            state.rateOfTurnDegPerSec + state.gyroBiasDegPerSec[2] + pseudoNoise(state.gyroNoiseDegPerSec, 3.3));

        if (state.magDropout)
        {
            s.magRaw = Vec3(0, 0, 0);
        }
        else
        {
            Vec3 clean = tiltVector(levelFrameField(state.headingDeg, state.horizontalFieldMagnitude, state.verticalFieldMagnitude), rollRad, pitchRad);
            s.magRaw = Vec3(
                clean.x * state.magSoftIronScale[0] + state.magHardIronBias[0] + pseudoNoise(state.magNoise, 4.4),
                clean.y * state.magSoftIronScale[1] + state.magHardIronBias[1] + pseudoNoise(state.magNoise, 5.5),
                clean.z * state.magSoftIronScale[2] + state.magHardIronBias[2] + pseudoNoise(state.magNoise, 6.6));
        }

        return s;
    }

    namespace Scenarios
    {
        SimulatorState level360Rotation(double timeSec, double periodSec)
        {
            SimulatorState s;
            double frac = fmod(timeSec / periodSec, 1.0);
            if (frac < 0)
                frac += 1.0;
            s.headingDeg = frac * 360.0;
            s.rateOfTurnDegPerSec = 360.0 / periodSec;
            return s;
        }

        SimulatorState rotation360With20DegHeel(double timeSec, double periodSec)
        {
            SimulatorState s = level360Rotation(timeSec, periodSec);
            s.rollDeg = 20.0;
            return s;
        }

        SimulatorState fixedHeadingChangingAttitude(double timeSec, double headingDeg)
        {
            SimulatorState s;
            s.headingDeg = headingDeg;
            // Slow sinusoidal roll/pitch, like a boat rolling/pitching at
            // anchor - amplitude chosen to stay well within normal sailing
            // angles.
            s.rollDeg = 15.0 * sin(timeSec * 0.5);
            s.pitchDeg = 5.0 * sin(timeSec * 0.3 + 1.0);
            return s;
        }

        SimulatorState hardIronOffset(double timeSec, double headingDeg)
        {
            SimulatorState s;
            s.headingDeg = headingDeg;
            s.magHardIronBias[0] = 8.0;
            s.magHardIronBias[1] = -5.0;
            s.magHardIronBias[2] = 2.0;
            return s;
        }

        SimulatorState ellipticalSoftIronDistortion(double timeSec, double headingDeg)
        {
            SimulatorState s;
            s.headingDeg = headingDeg;
            s.magSoftIronScale[0] = 1.4;
            s.magSoftIronScale[1] = 0.7;
            return s;
        }

        SimulatorState suddenMagneticDisturbance(double timeSec, double disturbanceStartSec)
        {
            SimulatorState s;
            s.headingDeg = 90.0;
            if (timeSec >= disturbanceStartSec)
            {
                // A large, sudden local disturbance (e.g. a steel hull
                // fitting or a speaker magnet moving nearby) - big enough
                // that magnitude-based disturbance detection (ImuMagMonitor)
                // should catch it.
                s.magHardIronBias[0] = 60.0;
                s.magHardIronBias[1] = 60.0;
            }
            return s;
        }

        SimulatorState slowGyroDrift(double timeSec, double driftDegPerSecPerSec)
        {
            SimulatorState s;
            s.headingDeg = 180.0;
            s.gyroBiasDegPerSec[2] = driftDegPerSecPerSec * timeSec;
            return s;
        }

        SimulatorState headingWrapThroughNorth(double timeSec, double periodSec)
        {
            SimulatorState s;
            // Oscillates across 0/360 rather than sweeping the full
            // circle - specifically exercises the wrap boundary.
            double frac = fmod(timeSec / periodSec, 1.0);
            if (frac < 0)
                frac += 1.0;
            s.headingDeg = fmod(350.0 + frac * 20.0, 360.0);
            return s;
        }

        SimulatorState magnetometerDropout(double timeSec, double dropoutStartSec)
        {
            SimulatorState s;
            s.headingDeg = 200.0;
            s.magDropout = (timeSec >= dropoutStartSec);
            return s;
        }
    }
}
