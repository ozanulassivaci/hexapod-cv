"""Regenerate transport/generated_constants.py and
firmware/include/GeneratedConstants.h from transport/constants.yaml.

Run manually after editing transport/constants.yaml:

    python scripts/gen_protocol_constants.py
    (or: make gen-protocol)

Deliberately not wired into any build step -- see docs/protocol.md Section 5.
The firmware C header is the "later" docs/protocol.md Section 5 referred to;
it covers only the six values that were already being manually kept in sync
by hand before firmware existed (protocol version, sequence width, the two
arm timeouts, link timeout, heartbeat interval) -- the safety-critical ones,
where a mismatch misfires a failsafe rather than just rejecting a command.
Command-field bounds (SERVO_COUNT, pulse ranges, etc.) stay Python-only by
design, per the existing note in transport/protocol.py, and are hand-copied
into firmware/include/Config.h with a cross-reference comment instead --
a mismatch there is a rejected command, not a misfired failsafe.
"""

import pathlib

import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SOURCE = REPO_ROOT / "transport" / "constants.yaml"
PY_OUTPUT = REPO_ROOT / "transport" / "generated_constants.py"
C_OUTPUT = REPO_ROOT / "firmware" / "include" / "GeneratedConstants.h"

PY_HEADER = '''"""GENERATED FILE -- do not edit by hand.

Source: transport/constants.yaml
Regenerate with: python scripts/gen_protocol_constants.py (or: make gen-protocol)
"""

'''

C_HEADER = """// GENERATED FILE -- do not edit by hand.
//
// Source: transport/constants.yaml
// Regenerate with: python scripts/gen_protocol_constants.py (or: make gen-protocol)
//
// This is a manual step, not a build hook -- see docs/protocol.md Section 5
// for why. Because it's manual, it can be forgotten; the runtime cross-check
// in transport/link.py (RobotLink.constants_warning) and its firmware-side
// counterpart (telemetry's link_timeout_s field) are what actually catch
// drift, by comparing the two sides' compiled-in values at connect time.
// This file being current is a convenience, not the safety property.

#pragma once

"""


def main() -> None:
    values = yaml.safe_load(SOURCE.read_text())

    py_lines = [PY_HEADER]
    py_lines.append(f"PROTOCOL_VERSION = {values['protocol_version']!r}\n")
    py_lines.append(f"SEQUENCE_BITS = {values['sequence_bits']!r}\n")
    py_lines.append("SEQUENCE_MODULUS = 2 ** SEQUENCE_BITS\n")
    py_lines.append(f"LINK_TIMEOUT_S = {values['link_timeout_s']!r}\n")
    py_lines.append(f"HEARTBEAT_INTERVAL_S = {values['heartbeat_interval_s']!r}\n")
    py_lines.append(
        f"CALIBRATION_ARM_TIMEOUT_S = {values['calibration_arm_timeout_s']!r}\n"
    )
    py_lines.append(f"BENCH_ARM_TIMEOUT_S = {values['bench_arm_timeout_s']!r}\n")
    PY_OUTPUT.write_text("".join(py_lines))
    print(f"wrote {PY_OUTPUT.relative_to(REPO_ROOT)}")

    sequence_bits = int(values["sequence_bits"])
    c_lines = [C_HEADER]
    c_lines.append(f"#define HX_PROTOCOL_VERSION {int(values['protocol_version'])}\n")
    c_lines.append(f"#define HX_SEQUENCE_BITS {sequence_bits}\n")
    c_lines.append(f"#define HX_SEQUENCE_MODULUS (1ULL << HX_SEQUENCE_BITS)\n")
    c_lines.append(f"#define HX_LINK_TIMEOUT_S {float(values['link_timeout_s'])}f\n")
    c_lines.append(
        f"#define HX_HEARTBEAT_INTERVAL_S {float(values['heartbeat_interval_s'])}f\n"
    )
    c_lines.append(
        f"#define HX_CALIBRATION_ARM_TIMEOUT_S {float(values['calibration_arm_timeout_s'])}f\n"
    )
    c_lines.append(
        f"#define HX_BENCH_ARM_TIMEOUT_S {float(values['bench_arm_timeout_s'])}f\n"
    )
    C_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    C_OUTPUT.write_text("".join(c_lines))
    print(f"wrote {C_OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
