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
