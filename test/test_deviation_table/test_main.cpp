#include <unity.h>
#include <math.h>
#include "ImuDeviationTable.h"
#include "ImuAngleMath.h"

void setUp(void) {}
void tearDown(void) {}

void test_empty_table_no_correction(void)
{
    DeviationTable t;
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, t.correctionDegFor(90.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 90.0, t.apply(90.0));
}

void test_single_entry_applies_everywhere(void)
{
    DeviationTable t;
    t.addEntry(90.0, 93.0); // measured 90, actual 93 -> +3 correction
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 3.0, t.correctionDegFor(90.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 3.0, t.correctionDegFor(200.0));
}

void test_exact_match_returns_exact_correction(void)
{
    DeviationTable t;
    t.addEntry(0.0, 2.0);
    t.addEntry(90.0, 85.0);
    t.addEntry(180.0, 182.0);
    t.addEntry(270.0, 273.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.0, t.correctionDegFor(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -5.0, t.correctionDegFor(90.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.0, t.correctionDegFor(180.0));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 3.0, t.correctionDegFor(270.0));
}

void test_interpolates_between_entries(void)
{
    DeviationTable t;
    t.addEntry(0.0, 0.0);   // 0 correction at 0
    t.addEntry(90.0, 96.0); // +6 correction at 90
    double mid = t.correctionDegFor(45.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 3.0, mid); // halfway between 0 and 90 -> halfway between 0 and +6
}

void test_interpolates_across_wrap(void)
{
    DeviationTable t;
    t.addEntry(350.0, 348.0); // -2 correction
    t.addEntry(10.0, 14.0);   // +4 correction
    // Halfway between 350 and 10 (through the wrap) is 0.
    double atZero = t.correctionDegFor(0.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.5, 1.0, atZero); // halfway between -2 and +4
}

void test_add_entry_replaces_close_existing(void)
{
    DeviationTable t;
    t.addEntry(90.0, 93.0);
    TEST_ASSERT_EQUAL_INT(1, t.size());
    t.addEntry(90.3, 95.0); // within 1 degree of the existing entry - should replace, not add
    TEST_ASSERT_EQUAL_INT(1, t.size());
    TEST_ASSERT_DOUBLE_WITHIN(1.0, 5.0, t.correctionDegFor(90.3));
}

void test_add_entry_rejects_when_full(void)
{
    DeviationTable t;
    for (int i = 0; i < DeviationTable::MAX_ENTRIES; i++)
    {
        // Spread entries far enough apart (>1 degree) that none get
        // treated as "close enough to replace".
        double h = (double)i * (360.0 / DeviationTable::MAX_ENTRIES);
        bool ok = t.addEntry(h, h);
        TEST_ASSERT_TRUE(ok);
    }
    TEST_ASSERT_EQUAL_INT(DeviationTable::MAX_ENTRIES, t.size());
    // One more, far from all existing entries - table is full, should fail.
    bool ok = t.addEntry(5.5, 5.5);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT(DeviationTable::MAX_ENTRIES, t.size());
}

void test_twelve_point_table_30_degree_spacing(void)
{
    DeviationTable t;
    for (int i = 0; i < 12; i++)
    {
        double h = i * 30.0;
        // A simple sinusoidal deviation pattern, plausible for a real
        // compass swing.
        double correction = 4.0 * sin(h * M_PI / 180.0);
        t.addEntry(h, ImuAngleMath::wrap360(h + correction));
    }
    TEST_ASSERT_EQUAL_INT(12, t.size());
    for (int i = 0; i < 12; i++)
    {
        double h = i * 30.0;
        double expected = 4.0 * sin(h * M_PI / 180.0);
        TEST_ASSERT_DOUBLE_WITHIN(0.5, expected, t.correctionDegFor(h));
    }
}

void test_clear_empties_table(void)
{
    DeviationTable t;
    t.addEntry(0, 5);
    t.addEntry(90, 92);
    t.clear();
    TEST_ASSERT_EQUAL_INT(0, t.size());
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, t.correctionDegFor(45.0));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_empty_table_no_correction);
    RUN_TEST(test_single_entry_applies_everywhere);
    RUN_TEST(test_exact_match_returns_exact_correction);
    RUN_TEST(test_interpolates_between_entries);
    RUN_TEST(test_interpolates_across_wrap);
    RUN_TEST(test_add_entry_replaces_close_existing);
    RUN_TEST(test_add_entry_rejects_when_full);
    RUN_TEST(test_twelve_point_table_30_degree_spacing);
    RUN_TEST(test_clear_empties_table);
    return UNITY_END();
}
