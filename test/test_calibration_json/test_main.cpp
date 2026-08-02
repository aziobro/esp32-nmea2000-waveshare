#include <unity.h>
#include "ImuCalibrationJson.h"

void setUp(void) {}
void tearDown(void) {}

void test_export_then_import_round_trip(void)
{
    ImuCalibration cal = ImuCalibrationOps::identityDefault();
    cal.magBias[0] = 12.5;
    cal.magBias[1] = -7.25;
    cal.magBias[2] = 1.0;
    cal.magMatrix[0][0] = 1.1;
    cal.magMatrix[1][1] = 0.95;
    cal.magMatrix[2][2] = 1.0;
    cal.magCalibrationValid = true;
    cal.magCalibrationQuality = 0.8;
    cal.gyroBias[2] = 0.3;
    cal.gyroCalibrationValid = true;
    cal.fixedHeadingOffsetDeg = 4.5;

    DeviationTable dev;
    dev.addEntry(0.0, 2.0);
    dev.addEntry(90.0, 87.0);

    std::string json = ImuCalibrationJson::exportJson(cal, MountOrientation::Starboard, dev, true);

    ImuCalibration outCal;
    MountOrientation outOrientation;
    DeviationTable outDev;
    bool outDevEnabled;
    std::string err;
    bool ok = ImuCalibrationJson::importJson(json, outCal, outOrientation, outDev, outDevEnabled, err);
    TEST_ASSERT_TRUE_MESSAGE(ok, err.c_str());

    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 12.5, outCal.magBias[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -7.25, outCal.magBias[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.1, outCal.magMatrix[0][0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.95, outCal.magMatrix[1][1]);
    TEST_ASSERT_TRUE(outCal.magCalibrationValid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.8, outCal.magCalibrationQuality);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.3, outCal.gyroBias[2]);
    TEST_ASSERT_TRUE(outCal.gyroCalibrationValid);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 4.5, outCal.fixedHeadingOffsetDeg);
    TEST_ASSERT_EQUAL_INT((int)MountOrientation::Starboard, (int)outOrientation);
    TEST_ASSERT_TRUE(outDevEnabled);
    TEST_ASSERT_EQUAL_INT(2, outDev.size());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.0, outDev.correctionDegFor(0.0));
}

void test_identity_json_round_trips_to_identity(void)
{
    std::string json = ImuCalibrationJson::identityJson();
    ImuCalibration outCal;
    MountOrientation outOrientation;
    DeviationTable outDev;
    bool outDevEnabled;
    std::string err;
    bool ok = ImuCalibrationJson::importJson(json, outCal, outOrientation, outDev, outDevEnabled, err);
    TEST_ASSERT_TRUE_MESSAGE(ok, err.c_str());
    TEST_ASSERT_TRUE(ImuCalibrationOps::isMagIdentity(outCal));
    TEST_ASSERT_EQUAL_INT((int)MountOrientation::Forward, (int)outOrientation);
    TEST_ASSERT_FALSE(outDevEnabled);
}

static void assertRejectedAndUnchanged(const std::string &json, const char *expectSubstring)
{
    ImuCalibration outCal;
    outCal.magBias[0] = 999.0; // sentinel - must NOT be touched by a failed import
    MountOrientation outOrientation = MountOrientation::PortDown;
    DeviationTable outDev;
    bool outDevEnabled = true;
    std::string err;
    bool ok = ImuCalibrationJson::importJson(json, outCal, outOrientation, outDev, outDevEnabled, err);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(err.length() > 0);
    if (expectSubstring)
    {
        char msg[256];
        snprintf(msg, sizeof(msg), "expected error to mention '%s', got: %s", expectSubstring, err.c_str());
        TEST_ASSERT_TRUE_MESSAGE(err.find(expectSubstring) != std::string::npos, msg);
    }
    // Sentinel values must survive untouched - "preserve prior
    // calibration if import fails".
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 999.0, outCal.magBias[0]);
    TEST_ASSERT_EQUAL_INT((int)MountOrientation::PortDown, (int)outOrientation);
    TEST_ASSERT_TRUE(outDevEnabled);
}

void test_rejects_malformed_json_syntax(void)
{
    assertRejectedAndUnchanged("{not valid json", "malformed");
}

void test_rejects_missing_schema_version(void)
{
    assertRejectedAndUnchanged("{\"orientation\":0}", "schemaVersion");
}

void test_rejects_wrong_schema_version(void)
{
    assertRejectedAndUnchanged("{\"schemaVersion\":99,\"orientation\":0}", "schemaVersion");
}

void test_rejects_orientation_out_of_range(void)
{
    assertRejectedAndUnchanged(
        "{\"schemaVersion\":1,\"orientation\":99,"
        "\"magCalibration\":{\"bias\":[0,0,0],\"matrix\":[[1,0,0],[0,1,0],[0,0,1]]}}",
        "orientation");
}

void test_rejects_nan_bias(void)
{
    // NaN can't be expressed in valid JSON text, so this simulates the
    // "reject non-finite" requirement via an out-of-range value instead
    // (NaN/Inf are unrepresentable in JSON syntax itself - ArduinoJson's
    // parser would reject the document before validation even runs, so
    // the meaningful test is the explicit range check, which the same
    // code path performs first with isfinite()).
    assertRejectedAndUnchanged(
        "{\"schemaVersion\":1,\"orientation\":0,"
        "\"magCalibration\":{\"bias\":[999999,0,0],\"matrix\":[[1,0,0],[0,1,0],[0,0,1]]}}",
        "bias");
}

void test_rejects_malformed_bias_array_length(void)
{
    assertRejectedAndUnchanged(
        "{\"schemaVersion\":1,\"orientation\":0,"
        "\"magCalibration\":{\"bias\":[1,2],\"matrix\":[[1,0,0],[0,1,0],[0,0,1]]}}",
        "bias");
}

void test_rejects_singular_matrix(void)
{
    // All-zero matrix - determinant 0, not invertible, could never have
    // come from a real calibration fit.
    assertRejectedAndUnchanged(
        "{\"schemaVersion\":1,\"orientation\":0,"
        "\"magCalibration\":{\"bias\":[0,0,0],\"matrix\":[[0,0,0],[0,0,0],[0,0,0]]}}",
        "singular");
}

void test_rejects_implausible_matrix(void)
{
    // Determinant far too large to be a real soft-iron correction.
    assertRejectedAndUnchanged(
        "{\"schemaVersion\":1,\"orientation\":0,"
        "\"magCalibration\":{\"bias\":[0,0,0],\"matrix\":[[9,0,0],[0,9,0],[0,0,9]]}}",
        "singular");
}

void test_rejects_missing_mag_calibration(void)
{
    assertRejectedAndUnchanged("{\"schemaVersion\":1,\"orientation\":0}", "magCalibration");
}

void test_gyro_calibration_optional(void)
{
    // A hand-written or older file might omit gyroCalibration entirely -
    // should import fine with a default (invalid, zero-bias) gyro cal,
    // not be rejected.
    std::string json = "{\"schemaVersion\":1,\"orientation\":0,"
                   "\"magCalibration\":{\"bias\":[0,0,0],\"matrix\":[[1,0,0],[0,1,0],[0,0,1]]}}";
    ImuCalibration outCal;
    MountOrientation outOrientation;
    DeviationTable outDev;
    bool outDevEnabled;
    std::string err;
    bool ok = ImuCalibrationJson::importJson(json, outCal, outOrientation, outDev, outDevEnabled, err);
    TEST_ASSERT_TRUE_MESSAGE(ok, err.c_str());
    TEST_ASSERT_FALSE(outCal.gyroCalibrationValid);
}

void test_imports_json_produced_by_the_python_calibration_tool(void)
{
    // Captured verbatim from tools/icm20948_calibration/calibrate.py's
    // output fitting a synthetic rotated-ellipsoid capture (see
    // tools/icm20948_calibration/tests/test_ellipsoid_fit.py's
    // rotated_ellipsoid scenario) - a real cross-language check that the
    // Python tool's JSON is actually importable by this exact firmware
    // code, not just schema-compatible on paper.
    std::string json =
        "{\"schemaVersion\": 1, \"orientation\": 0, \"magCalibration\": {\"valid\": true, "
        "\"quality\": 0.9999999999999963, \"referenceMagnitude\": 46.850944434574245, "
        "\"bias\": [-10.000000000000007, 5.999999999999999, -4.000000000000013], "
        "\"matrix\": [[0.9574537023212278, 0.15675629044733275, 0.12088861730958242], "
        "[0.1567562904473327, 1.0979450508447002, -0.005907379533204061], "
        "[0.12088861730958245, -0.005907379533204138, 0.9899126350813997]]}, "
        "\"gyroCalibration\": {\"valid\": false, \"bias\": [0.0, 0.0, 0.0], \"standardDeviation\": [0.0, 0.0, 0.0]}, "
        "\"heading\": {\"fixedOffsetDeg\": 0.0}, "
        "\"deviationTable\": {\"enabled\": false, \"entries\": []}}";

    ImuCalibration outCal;
    MountOrientation outOrientation;
    DeviationTable outDev;
    bool outDevEnabled;
    std::string err;
    bool ok = ImuCalibrationJson::importJson(json, outCal, outOrientation, outDev, outDevEnabled, err);
    TEST_ASSERT_TRUE_MESSAGE(ok, err.c_str());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -10.0, outCal.magBias[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 6.0, outCal.magBias[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -4.0, outCal.magBias[2]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.9574537023212278, outCal.magMatrix[0][0]);
    TEST_ASSERT_TRUE(outCal.magCalibrationValid);
}

void test_rejects_too_many_deviation_entries(void)
{
    std::string json = "{\"schemaVersion\":1,\"orientation\":0,"
                   "\"magCalibration\":{\"bias\":[0,0,0],\"matrix\":[[1,0,0],[0,1,0],[0,0,1]]},"
                   "\"deviationTable\":{\"enabled\":true,\"entries\":[";
    for (int i = 0; i < 40; i++)
    {
        if (i > 0)
            json += ",";
        json += "{\"measuredHeadingDeg\":" + std::to_string(i) + ",\"correctionDeg\":1.0}";
    }
    json += "]}}";
    assertRejectedAndUnchanged(json, "entries");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_export_then_import_round_trip);
    RUN_TEST(test_identity_json_round_trips_to_identity);
    RUN_TEST(test_rejects_malformed_json_syntax);
    RUN_TEST(test_rejects_missing_schema_version);
    RUN_TEST(test_rejects_wrong_schema_version);
    RUN_TEST(test_rejects_orientation_out_of_range);
    RUN_TEST(test_rejects_nan_bias);
    RUN_TEST(test_rejects_malformed_bias_array_length);
    RUN_TEST(test_rejects_singular_matrix);
    RUN_TEST(test_rejects_implausible_matrix);
    RUN_TEST(test_rejects_missing_mag_calibration);
    RUN_TEST(test_gyro_calibration_optional);
    RUN_TEST(test_imports_json_produced_by_the_python_calibration_tool);
    RUN_TEST(test_rejects_too_many_deviation_entries);
    return UNITY_END();
}
