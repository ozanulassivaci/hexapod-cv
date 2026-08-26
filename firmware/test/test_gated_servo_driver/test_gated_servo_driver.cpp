#include <unity.h>

#include <map>
#include <set>
#include <utility>

#include "Config.h"
#include "GatedServoDriver.h"

class FakeServoOutput : public ServoOutput {
public:
    void setPulse(uint8_t board, uint8_t channel, uint16_t pulseUs) override {
        lastPulse[{board, channel}] = pulseUs;
    }
    void release(uint8_t board, uint8_t channel) override {
        released.insert({board, channel});
        lastPulse.erase({board, channel});
    }

    std::map<std::pair<uint8_t, uint8_t>, uint16_t> lastPulse;
    std::set<std::pair<uint8_t, uint8_t>> released;
};

class FakeServoProfileStore : public ServoProfileStore {
public:
    ServoProfile get(uint8_t servoIndex) override {
        auto it = profiles.find(servoIndex);
        if (it != profiles.end()) return it->second;
        return ServoProfile::defaultFor(servoIndex);
    }
    void set(uint8_t servoIndex, const ServoProfile& profile) override {
        profiles[servoIndex] = profile;
    }

    std::map<uint8_t, ServoProfile> profiles;
};

void setUp(void) {}
void tearDown(void) {}

void test_applies_pulse_with_no_servo_index(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    bool applied = driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 1500, false, 0, &reason);

    TEST_ASSERT_TRUE(applied);
    auto key = std::make_pair<uint8_t, uint8_t>(PCA9685_ADDR_BOARD_A, 3);
    TEST_ASSERT_EQUAL(1500, output.lastPulse[key]);
}

void test_applies_pulse_within_recorded_limits(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    ServoProfile p = ServoProfile::defaultFor(5);
    p.hasMinPulse = true;
    p.minPulseUs = 1000;
    p.hasMaxPulse = true;
    p.maxPulseUs = 2000;
    store.set(5, p);
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    bool applied = driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 1500, true, 5, &reason);

    TEST_ASSERT_TRUE(applied);
    auto key = std::make_pair<uint8_t, uint8_t>(PCA9685_ADDR_BOARD_A, 3);
    TEST_ASSERT_EQUAL(1500, output.lastPulse[key]);
}

void test_refuses_pulse_below_recorded_min(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    ServoProfile p = ServoProfile::defaultFor(5);
    p.hasMinPulse = true;
    p.minPulseUs = 1000;
    store.set(5, p);
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    bool applied = driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 900, true, 5, &reason);

    TEST_ASSERT_FALSE(applied);
    TEST_ASSERT_NOT_NULL(reason);
    // The underlying hardware must never see this pulse at all -- not
    // clamped, refused outright.
    auto key = std::make_pair<uint8_t, uint8_t>(PCA9685_ADDR_BOARD_A, 3);
    TEST_ASSERT_EQUAL(0, output.lastPulse.count(key));
}

void test_refuses_pulse_above_recorded_max(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    ServoProfile p = ServoProfile::defaultFor(5);
    p.hasMaxPulse = true;
    p.maxPulseUs = 2000;
    store.set(5, p);
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    bool applied = driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 2100, true, 5, &reason);

    TEST_ASSERT_FALSE(applied);
    auto key = std::make_pair<uint8_t, uint8_t>(PCA9685_ADDR_BOARD_A, 3);
    TEST_ASSERT_EQUAL(0, output.lastPulse.count(key));
}

void test_boundary_values_are_accepted(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    ServoProfile p = ServoProfile::defaultFor(5);
    p.hasMinPulse = true;
    p.minPulseUs = 1000;
    p.hasMaxPulse = true;
    p.maxPulseUs = 2000;
    store.set(5, p);
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    TEST_ASSERT_TRUE(driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 1000, true, 5, &reason));
    TEST_ASSERT_TRUE(driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, 2000, true, 5, &reason));
}

void test_no_recorded_limits_allows_any_in_range_pulse(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;  // servo 5 never recorded -- defaults, no limits
    GatedServoDriver driver(output, store);

    const char* reason = nullptr;
    TEST_ASSERT_TRUE(driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, BENCH_PULSE_MIN_US, true, 5, &reason));
    TEST_ASSERT_TRUE(driver.commandPulse(PCA9685_ADDR_BOARD_A, 3, BENCH_PULSE_MAX_US, true, 5, &reason));
}

void test_release_forwards_to_output(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    GatedServoDriver driver(output, store);

    driver.release(PCA9685_ADDR_BOARD_A, 7);
    auto key = std::make_pair<uint8_t, uint8_t>(PCA9685_ADDR_BOARD_A, 7);
    TEST_ASSERT_EQUAL(1, output.released.count(key));
}

void test_release_all_covers_every_channel_on_both_boards(void) {
    FakeServoOutput output;
    FakeServoProfileStore store;
    GatedServoDriver driver(output, store);

    driver.releaseAll();

    TEST_ASSERT_EQUAL(2 * PCA9685_CHANNELS_PER_BOARD, output.released.size());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_applies_pulse_with_no_servo_index);
    RUN_TEST(test_applies_pulse_within_recorded_limits);
    RUN_TEST(test_refuses_pulse_below_recorded_min);
    RUN_TEST(test_refuses_pulse_above_recorded_max);
    RUN_TEST(test_boundary_values_are_accepted);
    RUN_TEST(test_no_recorded_limits_allows_any_in_range_pulse);
    RUN_TEST(test_release_forwards_to_output);
    RUN_TEST(test_release_all_covers_every_channel_on_both_boards);
    return UNITY_END();
}
