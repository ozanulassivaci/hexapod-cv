// Central constants. No magic numbers scattered elsewhere in the firmware
// (CLAUDE.md convention, applied here same as the Python side).
//
// GeneratedConstants.h covers the values that must byte-for-byte match the
// PC side or the failsafe misfires (protocol version, sequence width, both
// arm timeouts, link timeout, heartbeat interval) -- see docs/protocol.md
// Section 5. Everything below is either firmware-local (pins, I2C
// addresses, the control loop rate) or a hand-copied mirror of a
// transport/protocol.py command-field bound, where a mismatch means a
// rejected command, not a misfired failsafe (protocol.py has the same note
// at its own definitions) -- deliberately not in the shared codegen for
// that reason.
#pragma once

#include "GeneratedConstants.h"

// --- I2C / PCA9685 ----------------------------------------------------

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

#define PCA9685_ADDR_BOARD_A 0x40
#define PCA9685_ADDR_BOARD_B 0x41
#define PCA9685_CHANNELS_PER_BOARD 16
#define PCA9685_PWM_FREQ_HZ 50.0f

// PCA9685's internal RC oscillator is nominally 25MHz, but clone boards
// drift -- sometimes by a few percent -- which shifts every pulse width on
// that board by the same ratio. MEASURE THESE PER BOARD before trusting
// any pulse value; see docs/HOW_TO_USE.md's oscillator measurement
// procedure (a multimeter in Hz mode reading a commanded channel's actual
// output frequency, cross-checked against a servo tester's center
// reference). The two boards can drift differently even from the same
// batch -- don't assume the same corrected value applies to both.
//
// PLACEHOLDER -- both currently set to nominal. Replace with your measured
// values before relying on absolute pulse-width accuracy for anything
// (bench testing still works with the nominal value, it just means
// "1500us" might actually be a few percent off from that on an
// unmeasured board).
#define PCA9685_OSC_FREQ_BOARD_A_HZ 25000000.0f  // MEASURE ME
#define PCA9685_OSC_FREQ_BOARD_B_HZ 25000000.0f  // MEASURE ME

// --- Protocol command-field bounds (mirrors transport/protocol.py) -----

#define SERVO_COUNT 18
#define CALIBRATION_OFFSET_LIMIT_US 500
#define NEUTRAL_PULSE_US 1500
#define BENCH_PULSE_MIN_US 500
#define BENCH_PULSE_MAX_US 2500
#define HEALTH_NOTE_MAX_LEN 500

// AngleToPulse's neutral pulse for the tibia joint specifically -- the
// pulse that means "tibia dead straight" (kinematic zero). Coxa and
// femur are bipolar around NEUTRAL_PULSE_US (servo command 90 = straight
// leg's servo-degree neutral); the tibia horn is zero-based instead (0,
// not 90, is straight -- ANALYSIS.md Section 2). ANALYSIS.md flags that
// as an undocumented hardware coupling with "no way to verify or correct
// in software" -- true of the reference firmware, which wrote raw
// Servo::write() with no calibration layer at all. It doesn't apply
// here: this constant *is* that missing calibration point, decoupling
// kinematic zero from wherever the servo's own physical zero happens to
// land, the same way NEUTRAL_PULSE_US already does for coxa/femur.
//
// TARGET, not yet measured: docs/HOW_TO_USE.md's mounting procedure
// mounts every joint's horn with the servo commanded to its own 1500us
// center while the segment is held at that joint's gait-envelope
// midpoint -- for tibia, a ~77-degree fold from straight. Under that
// mounting, "dead straight" (tibia_servo = 0) works out to ~1500 -
// 77 degrees * kUsPerDeg =~ 645us. Carries the same up to +-7.2 degree
// (~80us) spline-tooth uncertainty as every other mounted joint here --
// MEASURE ME once assembled: command pulses near 645 and record the one
// where the tibia reads dead straight against the femur's own line, same
// procedure as the oscillator frequency measurement. Update this and
// transport/protocol.py's mirrored constant together if the measured
// value differs meaningfully.
#define TIBIA_NEUTRAL_PULSE_US 645

// The margin subtracted inward from each end of a servo's bench-recorded
// mechanical limit (ServoProfile.h's minDegFromNeutral/maxDegFromNeutral
// -- where printed parts actually collide) to get the *safe* limit gait
// output is enforced against (GatedServoDriver.h's
// LimitMode::Safe). Deliberately not applied to bench mode's own
// enforcement (LimitMode::Mechanical) -- exploration needs to reach the
// true mechanical edge to find it, not stop short of it by the margin.
// Mirrors robot/kinematics.py's SAFE_LIMIT_MARGIN_DEG.
#define SAFE_LIMIT_MARGIN_DEG 5.0f

// Fault flag bits, mirrors transport/protocol.py's FAULT_* constants.
// LINK_TIMEOUT, SERVO_FAULT (set when the gait control loop's computed
// pulse is refused by GatedServoDriver against a bench-recorded limit --
// see main.cpp's driveGaitOutputs), and IK_CLIP (set when the most
// recent gait tick needed to clamp a leg's D or a joint angle -- see
// GaitState::clippedThisTick) are the only bits this build ever sets;
// ESTOP and BROWNOUT are reserved so the wire format doesn't change when
// those fault sources exist later.
#define FAULT_LINK_TIMEOUT_BIT (1u << 0)
#define FAULT_ESTOP_BIT (1u << 1)
#define FAULT_SERVO_FAULT_BIT (1u << 2)
#define FAULT_BROWNOUT_BIT (1u << 3)
#define FAULT_IK_CLIP_BIT (1u << 4)

// --- Safe-state mode -----------------------------------------------------
//
// Build flag, not a runtime command or an inference from recorded servo
// profiles -- see the design discussion this was built from for the full
// argument (SafeState.h carries the short version). Defaults to 0
// (bench/release) because the two misconfigurations are not symmetric:
// flag=bench on an actually-assembled, standing robot means a fault
// releases a loaded joint and it drops; flag=assembled on an actually-
// bare bench servo means a fault just leaves an unloaded servo
// energized, holding whatever pulse it last had -- mildly wasteful, not
// damaging. Flip this deliberately, once, after assembly is actually
// done and before ever letting the robot stand unattended -- see
// docs/HOW_TO_USE.md.
#define ROBOT_ASSEMBLED 0

// --- Networking ---------------------------------------------------------

#define ROBOT_UDP_PORT 9000  // matches operator_config.yaml's link.udp.robot_port
#define UDP_RECV_BUFFER_SIZE 512
#define WIFI_CONNECT_TIMEOUT_MS 15000  // how long to try STA before falling back to AP
#define SERIAL_BAUD 115200

#define OTA_HOSTNAME "hexapod-cv"

// --- Control loop --------------------------------------------------------

#define CONTROL_LOOP_INTERVAL_MS 20  // 50Hz -- matches PCA9685's native PWM rate

// Firmware-side dwell protection is independent of the GUI's own
// bench.dwell_timeout_s (operator_config.yaml) on purpose -- "don't rely on
// the GUI to enforce dwell" means this must work even if the GUI never
// existed. No requirement that the two values match; this default just
// happens to equal the GUI's own default for consistency.
#define DWELL_TIMEOUT_S 8.0f

// --- NVS -----------------------------------------------------------------

#define NVS_NAMESPACE "hexapod"
