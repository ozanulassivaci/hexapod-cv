"""Regenerate transport/generated_constants.py from transport/constants.yaml.

Run manually after editing transport/constants.yaml:

    python scripts/gen_protocol_constants.py
    (or: make gen-protocol)

Deliberately not wired into any build step -- see docs/protocol.md Section 5.
"""

import pathlib

import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SOURCE = REPO_ROOT / "transport" / "constants.yaml"
OUTPUT = REPO_ROOT / "transport" / "generated_constants.py"

HEADER = '''"""GENERATED FILE -- do not edit by hand.

Source: transport/constants.yaml
Regenerate with: python scripts/gen_protocol_constants.py (or: make gen-protocol)
"""

'''


def main() -> None:
    values = yaml.safe_load(SOURCE.read_text())

    lines = [HEADER]
    lines.append(f"PROTOCOL_VERSION = {values['protocol_version']!r}\n")
    lines.append(f"SEQUENCE_BITS = {values['sequence_bits']!r}\n")
    lines.append("SEQUENCE_MODULUS = 2 ** SEQUENCE_BITS\n")
    lines.append(f"LINK_TIMEOUT_S = {values['link_timeout_s']!r}\n")
    lines.append(f"HEARTBEAT_INTERVAL_S = {values['heartbeat_interval_s']!r}\n")
    lines.append(
        f"CALIBRATION_ARM_TIMEOUT_S = {values['calibration_arm_timeout_s']!r}\n"
    )

    OUTPUT.write_text("".join(lines))
    print(f"wrote {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
