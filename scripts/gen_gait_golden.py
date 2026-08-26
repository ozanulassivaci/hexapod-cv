"""Generates tests/fixtures/gait_golden.json from robot/gait.py.

Manual step, run after changing robot/gait.py's math -- same "manual
codegen, not a hook" philosophy as scripts/gen_protocol_constants.py and
scripts/gen_kinematics_golden.py (see docs/protocol.md Section 5). This
is the cross-check artifact between robot/gait.py and
firmware/lib/core/Gait.cpp -- see reference/ANALYSIS.md Section 7. Both
tests/test_gait.py and firmware/test/test_gait/ load this file and assert
against it.

    python scripts/gen_gait_golden.py
"""

import json
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from robot.gait import GaitState, step  # noqa: E402

OUT_PATH = _REPO_ROOT / "tests" / "fixtures" / "gait_golden.json"

DT_S = 0.02  # matches firmware's CONTROL_LOOP_INTERVAL_MS (50 Hz)
TICKS = 60
RECORD_EVERY = 5

SCENARIOS = {
    "walk_forward": {"vx": 1.0, "vy": 0.0, "speed": 50.0, "rotation": 0.0, "body_height": 50.0},
    "strafe_right": {"vx": 0.0, "vy": 1.0, "speed": 40.0, "rotation": 0.0, "body_height": 50.0},
    "turn_in_place": {"vx": 0.0, "vy": 0.0, "speed": 60.0, "rotation": 1.0, "body_height": 30.0},
    "diagonal_unnormalized": {"vx": 1.0, "vy": 1.0, "speed": 70.0, "rotation": 0.0, "body_height": 60.0},
}


def _angles_dict(angles):
    return {"coxa_deg": angles.coxa_deg, "femur_deg": angles.femur_deg, "tibia_deg": angles.tibia_deg}


def main() -> None:
    scenarios_out = []
    for name, cmd in SCENARIOS.items():
        state = GaitState.initial(body_height=cmd["body_height"])
        ticks_out = []
        for tick in range(1, TICKS + 1):
            state = step(
                state,
                DT_S,
                vx=cmd["vx"],
                vy=cmd["vy"],
                speed=cmd["speed"],
                rotation=cmd["rotation"],
                body_height=cmd["body_height"],
            )
            if tick % RECORD_EVERY == 0 or tick == TICKS:
                ticks_out.append(
                    {
                        "tick": tick,
                        "phase": state.phase,
                        "leg_angles": [_angles_dict(a) for a in state.leg_angles],
                    }
                )
        scenarios_out.append({"name": name, "command": cmd, "dt_s": DT_S, "ticks": ticks_out})

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps({"scenarios": scenarios_out}, indent=2) + "\n")
    print(f"wrote {len(scenarios_out)} scenarios to {OUT_PATH}")


if __name__ == "__main__":
    main()
