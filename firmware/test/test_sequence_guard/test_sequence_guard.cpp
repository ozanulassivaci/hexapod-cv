// Mirrors tests/test_protocol.py's sequence tests -- same cases, same
// reasoning, different language.
#include <unity.h>

#include "SequenceGuard.h"

void setUp(void) {}
void tearDown(void) {}

void test_simple_increment(void) {
    TEST_ASSERT_TRUE(sequenceIsNewer(1, 0));
    TEST_ASSERT_FALSE(sequenceIsNewer(0, 1));
}

void test_equal_is_not_newer(void) {
    TEST_ASSERT_FALSE(sequenceIsNewer(5, 5));
}

void test_handles_wraparound(void) {
    uint32_t reference = 0xFFFFFFFFu;  // 2^32 - 1
    uint32_t candidate = 0;
    TEST_ASSERT_TRUE(sequenceIsNewer(candidate, reference));
    TEST_ASSERT_FALSE(sequenceIsNewer(reference, candidate));
}

void test_far_older_is_not_newer(void) {
    TEST_ASSERT_FALSE(sequenceIsNewer(10, 1000));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_simple_increment);
    RUN_TEST(test_equal_is_not_newer);
    RUN_TEST(test_handles_wraparound);
    RUN_TEST(test_far_older_is_not_newer);
    return UNITY_END();
}
