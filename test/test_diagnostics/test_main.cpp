#include <unity.h>
#include <string.h>
#include "ImuDiagnostics.h"

void setUp(void) {}
void tearDown(void) {}

static int countCommas(const char *s)
{
    int n = 0;
    for (; *s; s++)
        if (*s == ',')
            n++;
    return n;
}

void test_header_and_row_have_same_column_count(void)
{
    DiagnosticSample s;
    char buf[600];
    int n = ImuDiagnostics::formatCsvRow(s, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    int headerCommas = countCommas(ImuDiagnostics::csvHeader());
    int rowCommas = countCommas(buf);
    TEST_ASSERT_EQUAL_INT(headerCommas, rowCommas);
    // 43 fields listed in the project spec -> 42 commas.
    TEST_ASSERT_EQUAL_INT(42, headerCommas);
}

void test_row_contains_expected_values(void)
{
    DiagnosticSample s;
    s.timestampMs = 12345;
    s.sampleSequence = 7;
    s.accelRaw = Vec3(1.0, 2.0, 3.0);
    s.outputHeadingDeg = 271.5;
    s.activeSource = HeadingSource::SoftwareFusion;
    s.headingQuality = HeadingQuality::Good;
    s.rejectionFlags = 0;
    char buf[600];
    int n = ImuDiagnostics::formatCsvRow(s, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_TRUE(strstr(buf, "12345,7,") == buf);
    TEST_ASSERT_NOT_NULL(strstr(buf, "software_9axis_fusion"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "good"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "271.50"));
}

void test_truncation_detected_with_small_buffer(void)
{
    DiagnosticSample s;
    char tinyBuf[8];
    int n = ImuDiagnostics::formatCsvRow(s, tinyBuf, sizeof(tinyBuf));
    TEST_ASSERT_EQUAL_INT(-1, n);
}

void test_default_sample_has_no_dmp_quaternion_garbage(void)
{
    // A default-constructed sample's quaternion should be the identity
    // (w=1, x=y=z=0), not zero-initialized (which wouldn't be a valid
    // rotation) - guards against a lazy struct default silently
    // producing bad data before the first real DMP sample arrives.
    DiagnosticSample s;
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, s.dmpQ0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, s.dmpQ1);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_header_and_row_have_same_column_count);
    RUN_TEST(test_row_contains_expected_values);
    RUN_TEST(test_truncation_detected_with_small_buffer);
    RUN_TEST(test_default_sample_has_no_dmp_quaternion_garbage);
    return UNITY_END();
}
