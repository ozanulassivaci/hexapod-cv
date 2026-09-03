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
    TEST_ASSERT_TRUE(watchdog.observePacket(1, 0.0f) == Observation::Accepted);
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
    Observation obs = watchdog.observePacket(3, 0.5f);
    TEST_ASSERT_TRUE(obs == Observation::Stale);
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(1.0f));  // still measured from t=0, not t=0.5
}

void test_duplicate_packet_does_not_reset_watchdog(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(10, 0.0f);
    Observation obs = watchdog.observePacket(10, 0.5f);
    TEST_ASSERT_TRUE(obs == Observation::Stale);
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
    Observation obs = watchdog.observePacket(0, 0.1f);  // wrapped, but genuinely newer
    TEST_ASSERT_TRUE(obs == Observation::Accepted);
    TEST_ASSERT_FALSE(watchdog.hasTimedOut(0.5f));
}

// --- the re-baseline rule ------------------------------------------------
//
// Mirrors tests/test_link_watchdog.py exactly -- every case there should
// exist here and vice versa. These are the ones that fail against the
// pre-fix rule, where a restarted PC counter was rejected indefinitely.

void test_last_accepted_seq_tracks_accepted_packets(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(12345, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(12345, watchdog.lastAcceptedSeq());
    watchdog.observePacket(3, 0.1f);  // stale -- must not move the baseline
    TEST_ASSERT_EQUAL_UINT32(12345, watchdog.lastAcceptedSeq());
}

void test_stale_drops_are_counted(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(10, 0.0f);
    TEST_ASSERT_EQUAL_UINT32(0, watchdog.staleDropCount());
    watchdog.observePacket(9, 0.1f);
    watchdog.observePacket(10, 0.2f);
    TEST_ASSERT_EQUAL_UINT32(2, watchdog.staleDropCount());
}

void test_restarted_client_rejected_while_stream_is_live(void) {
    LinkWatchdog watchdog(1.0f);
    for (uint32_t seq = 0; seq < 600; ++seq) {
        watchdog.observePacket(seq, seq * 0.1f);
    }
    TEST_ASSERT_TRUE(watchdog.observePacket(0, 59.95f) == Observation::Stale);
}

void test_restarted_client_accepted_after_timeout(void) {
    LinkWatchdog watchdog(1.0f);
    for (uint32_t seq = 0; seq < 600; ++seq) {
        watchdog.observePacket(seq, seq * 0.1f);
    }
    TEST_ASSERT_TRUE(watchdog.observePacket(0, 61.0f) == Observation::NewSession);
    TEST_ASSERT_EQUAL_UINT32(0, watchdog.lastAcceptedSeq());
}

void test_new_session_is_not_counted_as_a_drop(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(600, 0.0f);
    watchdog.observePacket(0, 5.0f);
    TEST_ASSERT_EQUAL_UINT32(0, watchdog.staleDropCount());
}

void test_reorder_protection_returns_after_a_re_baseline(void) {
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(600, 0.0f);
    TEST_ASSERT_TRUE(watchdog.observePacket(0, 5.0f) == Observation::NewSession);
    TEST_ASSERT_TRUE(watchdog.observePacket(1, 5.1f) == Observation::Accepted);
    TEST_ASSERT_TRUE(watchdog.observePacket(0, 5.2f) == Observation::Stale);
}

void test_re_baseline_triggers_at_exactly_the_timeout_boundary(void) {
    // observePacket is defined in terms of hasTimedOut precisely so the
    // two cannot disagree about where the boundary sits.
    LinkWatchdog watchdog(1.0f);
    watchdog.observePacket(10, 0.0f);
    TEST_ASSERT_TRUE(watchdog.hasTimedOut(1.0f));
    TEST_ASSERT_TRUE(watchdog.observePacket(5, 1.0f) == Observation::NewSession);
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
    RUN_TEST(test_last_accepted_seq_tracks_accepted_packets);
    RUN_TEST(test_stale_drops_are_counted);
    RUN_TEST(test_restarted_client_rejected_while_stream_is_live);
    RUN_TEST(test_restarted_client_accepted_after_timeout);
    RUN_TEST(test_new_session_is_not_counted_as_a_drop);
    RUN_TEST(test_reorder_protection_returns_after_a_re_baseline);
    RUN_TEST(test_re_baseline_triggers_at_exactly_the_timeout_boundary);
    return UNITY_END();
}
