#include "ServoMap.h"

namespace {
constexpr ServoMapEntry entry(uint8_t legIndex, JointType joint) {
    const uint8_t board = (legIndex < 3) ? PCA9685_ADDR_BOARD_A : PCA9685_ADDR_BOARD_B;
    const uint8_t legWithinBoard = legIndex % 3;
    const uint8_t channel = legWithinBoard * 3 + static_cast<uint8_t>(joint);
    return ServoMapEntry{legIndex, joint, board, channel};
}
}  // namespace

const ServoMapEntry kServoMap[SERVO_COUNT] = {
    entry(0, JointType::Coxa),  entry(0, JointType::Femur), entry(0, JointType::Tibia),
    entry(1, JointType::Coxa),  entry(1, JointType::Femur), entry(1, JointType::Tibia),
    entry(2, JointType::Coxa),  entry(2, JointType::Femur), entry(2, JointType::Tibia),
    entry(3, JointType::Coxa),  entry(3, JointType::Femur), entry(3, JointType::Tibia),
    entry(4, JointType::Coxa),  entry(4, JointType::Femur), entry(4, JointType::Tibia),
    entry(5, JointType::Coxa),  entry(5, JointType::Femur), entry(5, JointType::Tibia),
};
