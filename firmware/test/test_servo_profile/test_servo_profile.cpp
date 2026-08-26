#include <unity.h>

#include <cstring>

#include "ServoProfile.h"

void setUp(void) {}
void tearDown(void) {}

void test_default_has_no_recorded_limits(void) {
    ServoProfile p = ServoProfile::defaultFor(7);
    TEST_ASSERT_EQUAL_UINT8(7, p.servoIndex);
    TEST_ASSERT_EQUAL_INT16(0, p.offsetUs);
    TEST_ASSERT_EQUAL_INT8(1, p.sign);
    TEST_ASSERT_FALSE(p.hasMinPulse);
    TEST_ASSERT_FALSE(p.hasMaxPulse);
    TEST_ASSERT_EQUAL_STRING("", p.note);
}

void test_merge_offset_sign_preserves_limits_and_note(void) {
    ServoProfile existing = ServoProfile::defaultFor(5);
    existing.hasMinPulse = true;
    existing.minPulseUs = 950;
    existing.hasMaxPulse = true;
    existing.maxPulseUs = 2050;
    std::strcpy(existing.note, "buzzes at low end");

    ServoProfile updated = mergeOffsetSign(existing, 30, -1);

    TEST_ASSERT_EQUAL_INT16(30, updated.offsetUs);
    TEST_ASSERT_EQUAL_INT8(-1, updated.sign);
    TEST_ASSERT_TRUE(updated.hasMinPulse);
    TEST_ASSERT_EQUAL_UINT16(950, updated.minPulseUs);
    TEST_ASSERT_TRUE(updated.hasMaxPulse);
    TEST_ASSERT_EQUAL_UINT16(2050, updated.maxPulseUs);
    TEST_ASSERT_EQUAL_STRING("buzzes at low end", updated.note);
}

void test_merge_limit_min_preserves_offset_sign_and_note(void) {
    ServoProfile existing = ServoProfile::defaultFor(3);
    existing.offsetUs = 15;
    existing.sign = -1;
    std::strcpy(existing.note, "dead");

    ServoProfile updated;
    bool ok = mergeLimit(existing, LimitBound::Min, 1000, updated);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(updated.hasMinPulse);
    TEST_ASSERT_EQUAL_UINT16(1000, updated.minPulseUs);
    TEST_ASSERT_FALSE(updated.hasMaxPulse);
    TEST_ASSERT_EQUAL_INT16(15, updated.offsetUs);
    TEST_ASSERT_EQUAL_INT8(-1, updated.sign);
    TEST_ASSERT_EQUAL_STRING("dead", updated.note);
}

void test_merge_limit_max_then_min_round_trips(void) {
    ServoProfile p = ServoProfile::defaultFor(0);
    ServoProfile afterMax;
    TEST_ASSERT_TRUE(mergeLimit(p, LimitBound::Max, 2050, afterMax));
    ServoProfile afterMin;
    TEST_ASSERT_TRUE(mergeLimit(afterMax, LimitBound::Min, 950, afterMin));
    TEST_ASSERT_EQUAL_UINT16(950, afterMin.minPulseUs);
    TEST_ASSERT_EQUAL_UINT16(2050, afterMin.maxPulseUs);
}

void test_merge_limit_rejects_min_at_or_past_existing_max(void) {
    ServoProfile p = ServoProfile::defaultFor(0);
    ServoProfile afterMax;
    mergeLimit(p, LimitBound::Max, 2000, afterMax);

    ServoProfile out;
    TEST_ASSERT_FALSE(mergeLimit(afterMax, LimitBound::Min, 2000, out));  // equal
    TEST_ASSERT_FALSE(mergeLimit(afterMax, LimitBound::Min, 2100, out));  // past
}

void test_merge_limit_rejects_max_at_or_before_existing_min(void) {
    ServoProfile p = ServoProfile::defaultFor(0);
    ServoProfile afterMin;
    mergeLimit(p, LimitBound::Min, 1000, afterMin);

    ServoProfile out;
    TEST_ASSERT_FALSE(mergeLimit(afterMin, LimitBound::Max, 1000, out));  // equal
    TEST_ASSERT_FALSE(mergeLimit(afterMin, LimitBound::Max, 900, out));   // before
}

void test_merge_limit_first_recording_has_nothing_to_conflict_with(void) {
    ServoProfile p = ServoProfile::defaultFor(0);
    ServoProfile out;
    TEST_ASSERT_TRUE(mergeLimit(p, LimitBound::Min, 2400, out));  // no max recorded yet
    TEST_ASSERT_TRUE(mergeLimit(p, LimitBound::Max, 600, out));   // no min recorded yet
}

void test_merge_note_preserves_offset_sign_and_limits(void) {
    ServoProfile existing = ServoProfile::defaultFor(9);
    existing.offsetUs = -20;
    existing.hasMinPulse = true;
    existing.minPulseUs = 1000;

    ServoProfile updated = mergeNote(existing, "fine");

    TEST_ASSERT_EQUAL_STRING("fine", updated.note);
    TEST_ASSERT_EQUAL_INT16(-20, updated.offsetUs);
    TEST_ASSERT_TRUE(updated.hasMinPulse);
    TEST_ASSERT_EQUAL_UINT16(1000, updated.minPulseUs);
}

void test_merge_note_truncates_defensively(void) {
    // Protocol decode already enforces HEALTH_NOTE_MAX_LEN before this is
    // ever called; this checks the defensive copy doesn't overflow even if
    // some future caller forgets that.
    char oversized[HEALTH_NOTE_MAX_LEN + 100];
    std::memset(oversized, 'x', sizeof(oversized) - 1);
    oversized[sizeof(oversized) - 1] = '\0';

    ServoProfile p = ServoProfile::defaultFor(0);
    ServoProfile updated = mergeNote(p, oversized);

    TEST_ASSERT_EQUAL(HEALTH_NOTE_MAX_LEN, std::strlen(updated.note));
    TEST_ASSERT_EQUAL_CHAR('\0', updated.note[HEALTH_NOTE_MAX_LEN]);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_default_has_no_recorded_limits);
    RUN_TEST(test_merge_offset_sign_preserves_limits_and_note);
    RUN_TEST(test_merge_limit_min_preserves_offset_sign_and_note);
    RUN_TEST(test_merge_limit_max_then_min_round_trips);
    RUN_TEST(test_merge_limit_rejects_min_at_or_past_existing_max);
    RUN_TEST(test_merge_limit_rejects_max_at_or_before_existing_min);
    RUN_TEST(test_merge_limit_first_recording_has_nothing_to_conflict_with);
    RUN_TEST(test_merge_note_preserves_offset_sign_and_limits);
    RUN_TEST(test_merge_note_truncates_defensively);
    return UNITY_END();
}
