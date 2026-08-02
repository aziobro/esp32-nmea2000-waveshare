#include "ImuCalibration.h"
#include <math.h>

namespace ImuCalibrationOps
{
    Vec3 applyMag(const Vec3 &raw, const ImuCalibration &cal)
    {
        Vec3 d(raw.x - cal.magBias[0], raw.y - cal.magBias[1], raw.z - cal.magBias[2]);
        const auto &A = cal.magMatrix;
        return Vec3(
            A[0][0] * d.x + A[0][1] * d.y + A[0][2] * d.z,
            A[1][0] * d.x + A[1][1] * d.y + A[1][2] * d.z,
            A[2][0] * d.x + A[2][1] * d.y + A[2][2] * d.z);
    }

    Vec3 applyAccel(const Vec3 &raw, const ImuCalibration &cal)
    {
        return Vec3(
            (raw.x - cal.accelBias[0]) * cal.accelScale[0],
            (raw.y - cal.accelBias[1]) * cal.accelScale[1],
            (raw.z - cal.accelBias[2]) * cal.accelScale[2]);
    }

    Vec3 applyGyro(const Vec3 &raw, const ImuCalibration &cal)
    {
        return Vec3(raw.x - cal.gyroBias[0], raw.y - cal.gyroBias[1], raw.z - cal.gyroBias[2]);
    }

    ImuCalibration identityDefault()
    {
        return ImuCalibration();
    }

    ImuCalibration migrateFromLegacy(double legacyMagXOff, double legacyMagYOff, double legacyHeadingOffsetDeg)
    {
        ImuCalibration cal;
        cal.magBias[0] = legacyMagXOff;
        cal.magBias[1] = legacyMagYOff;
        cal.magBias[2] = 0;
        // magMatrix stays identity - the legacy path never applied a
        // soft-iron correction.
        cal.fixedHeadingOffsetDeg = legacyHeadingOffsetDeg;
        cal.magCalibrationValid = (legacyMagXOff != 0.0 || legacyMagYOff != 0.0);
        cal.magCalibrationQuality = cal.magCalibrationValid ? 0.5 : 0.0; // unknown quality, legacy 2D-only fit
        cal.calibrationSequence = cal.magCalibrationValid ? 1 : 0;
        return cal;
    }

    bool isMagIdentity(const ImuCalibration &cal, double tolerance)
    {
        for (int i = 0; i < 3; i++)
        {
            if (fabs(cal.magBias[i]) > tolerance)
                return false;
            for (int j = 0; j < 3; j++)
            {
                double expected = (i == j) ? 1.0 : 0.0;
                if (fabs(cal.magMatrix[i][j] - expected) > tolerance)
                    return false;
            }
        }
        return true;
    }
}
