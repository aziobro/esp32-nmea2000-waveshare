#include <unity.h>
#include "ImuCalibration.h"

void setUp(void) {}
void tearDown(void) {}

void test_identity_default_is_safe(void)
{
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    TEST_ASSERT_TRUE(ImuCalibrationOps::isMagIdentity(cal));
    TEST_ASSERT_FALSE(cal.magCalibrationValid);
    TEST_ASSERT_FALSE(cal.accelCalibrationValid);
    TEST_ASSERT_FALSE(cal.gyroCalibrationValid);
    // Applying an identity calibration must be a no-op.
    Vec3 raw(12.0, -5.0, 3.0);
    Vec3 out = ImuCalibrationOps::applyMag(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.x, out.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.y, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.z, out.z);
}

void test_apply_mag_hard_iron_only(void)
{
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magBias[0] = 10.0;
    cal.magBias[1] = -3.0;
    cal.magBias[2] = 0.0;
    Vec3 raw(15.0, 2.0, 7.0);
    Vec3 out = ImuCalibrationOps::applyMag(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.0, out.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.0, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 7.0, out.z);
}

void test_apply_mag_full_matrix(void)
{
    // A diagonal soft-iron scale (elliptical distortion correction) plus
    // hard-iron bias - the general case a real ellipsoid fit would produce.
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magBias[0] = 5.0;
    cal.magBias[1] = 0.0;
    cal.magBias[2] = 0.0;
    cal.magMatrix[0][0] = 2.0; // X axis was reading half-scale
    cal.magMatrix[1][1] = 1.0;
    cal.magMatrix[2][2] = 1.0;
    Vec3 raw(10.0, 3.0, -1.0);
    Vec3 out = ImuCalibrationOps::applyMag(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0, out.x); // (10-5)*2
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 3.0, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.0, out.z);
}

void test_apply_mag_off_diagonal_matrix(void)
{
    // A genuinely non-diagonal soft-iron matrix (cross-axis coupling) -
    // confirms applyMag does a real matrix multiply, not just per-axis
    // scaling.
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magMatrix[0][0] = 1.0;
    cal.magMatrix[0][1] = 0.5;
    cal.magMatrix[1][1] = 1.0;
    Vec3 raw(2.0, 4.0, 0.0);
    Vec3 out = ImuCalibrationOps::applyMag(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, out.x); // 1*2 + 0.5*4
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 4.0, out.y);
}

void test_apply_accel_scale_and_bias(void)
{
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.accelBias[0] = 0.1;
    cal.accelScale[1] = 1.05;
    Vec3 raw(1.1, 0.98, -9.8);
    Vec3 out = ImuCalibrationOps::applyAccel(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, out.x);       // (1.1-0.1)*1
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.029, out.y);     // (0.98-0)*1.05
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -9.8, out.z);
}

void test_apply_gyro_bias(void)
{
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.gyroBias[2] = 1.5; // stationary Z drift
    Vec3 raw(0.1, -0.2, 1.6);
    Vec3 out = ImuCalibrationOps::applyGyro(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.1, out.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.2, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.1, out.z);
}

// Config migration: the currently-deployed unit's saved icmMagXOff/
// icmMagYOff/icmHdgOff values must produce EXACTLY the same corrected
// output as the pre-rewrite code did, or an already-tuned installation
// would regress on upgrade.
void test_migrate_from_legacy_reproduces_old_behavior(void)
{
    double legacyMagXOff = 42.0, legacyMagYOff = -17.0, legacyHdgOff = 3.5;
    ImuCalibration cal = ImuCalibrationOps::migrateFromLegacy(legacyMagXOff, legacyMagYOff, legacyHdgOff);
    TEST_ASSERT_TRUE(cal.magCalibrationValid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, legacyHdgOff, cal.fixedHeadingOffsetDeg);

    // The pre-rewrite formula was: mx = magX - magXOffset; my = magY - magYOffset;
    // mz unchanged. Confirm applyMag matches that exactly.
    Vec3 raw(100.0, 50.0, -30.0);
    Vec3 out = ImuCalibrationOps::applyMag(raw, cal);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.x - legacyMagXOff, out.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.y - legacyMagYOff, out.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, raw.z, out.z); // Z was never hard-iron corrected
}

void test_migrate_from_legacy_zero_offsets_is_uncalibrated(void)
{
    // A unit that never ran the legacy calibration (offsets still at
    // their 0 default) should migrate to "not calibrated", not silently
    // claim validity.
    ImuCalibration cal = ImuCalibrationOps::migrateFromLegacy(0.0, 0.0, 0.0);
    TEST_ASSERT_FALSE(cal.magCalibrationValid);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_identity_default_is_safe);
    RUN_TEST(test_apply_mag_hard_iron_only);
    RUN_TEST(test_apply_mag_full_matrix);
    RUN_TEST(test_apply_mag_off_diagonal_matrix);
    RUN_TEST(test_apply_accel_scale_and_bias);
    RUN_TEST(test_apply_gyro_bias);
    RUN_TEST(test_migrate_from_legacy_reproduces_old_behavior);
    RUN_TEST(test_migrate_from_legacy_zero_offsets_is_uncalibrated);
    return UNITY_END();
}
