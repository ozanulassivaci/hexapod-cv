#include <unity.h>

#include "LinkWatchdog.h"

void setUp(void) {}
void tearDown(void) {}

void test_times_out_before_any_packet_ever_arrives(void) {
    // Deliberate: boot state and failsafe state are the same state. See
    // LinkWatchdog.h's design note and SafeState.h.
    LinkWatchdog watchdog(1.0f);
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(0.0f));
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(1000.0f));
    TEST_ASSERT_FALSE(watchdog.hasEverReceivedPacket());
}

void test_accepting_a_packet_clears_timeout(void) {
    LinkWatchdog watchdog(1.0f);
    TEST_ASSERT_TRUE(watchdog.observePacket(1, 0.0f));
    TEST_ASSERT_FALSE(watchdog.hasTimedOut(0.0f));
    TEST_ASSERT_TRUE(watchdog.hasEverReceivedPacket());
}

void test_times_out_after_configured_window(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(1, 0.0f);
    TEST_ASSERT_FALSE(watchdog.hasTimedOut(0.99f));
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(1.0f));
}

void test_stale_packet_does_not_reset_watchdog(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(10, 0.0f);
    // An older/duplicate seq arriving later must not count as fresh --
    // same latest-wins rule as everywhere else in this project.
    bool accepted = watchdog.observePacket(3, 0.5f);
    TEST_ASSERT_FALSE(accepted);
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(1.0f));  // still measured from t=0, not t=0.5
}

void test_duplicate_packet_does_not_reset_watchdog(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(10, 0.0f);
    bool accepted = watchdog.observePacket(10, 0.5f);
    TEST_ASSERT_FALSE(accepted);
}

void test_newer_packet_resets_the_clock(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(1, 0.0f);
    watchdog.observePacket(2, 0.9f);
    TEST_ASSERT_FALSE(watchdog.hasTimedOut(1.5f));  // 0.6s since the t=0.9 packet, not 1.5s since t=0
}

void test_seconds_since_last_packet(void) {
    LinkWatchdog watchdog(1.0f);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, watchdog.secondsSinceLastPacket(5.0f));
    watchdog.observePacket(1, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, watchdog.secondsSinceLastPacket(5.0f));
}

void test_wraparound_packet_is_still_accepted(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(0xFFFFFFFFu, 0.0f);
    bool accepted = watchdog.observePacket(0, 0.1f);  // wrapped, but genuinely newer
    TEST_ASSERT_TRUE(accepted);
    TEST_ASSERT_FALSE(watchdog.hasTimedOut(0.5f));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_times_out_before_any_packet_ever_arrives);
    RUN_TEST(test_accepting_a_packet_clears_timeout);
    RUN_TEST(test_times_out_after_configured_window);
    RUN_TEST(test_stale_packet_does_not_reset_watchdog);
    RUN_TEST(test_duplicate_packet_does_not_reset_watchdog);
    RUN_TEST(test_newer_packet_resets_the_clock);
    RUN_TEST(test_seconds_since_last_packet);
    RUN_TEST(test_wraparound_packet_is_still_accepted);
    return UNITY_END();
}
