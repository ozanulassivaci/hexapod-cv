"""WASD/QE -> walk/turn intent, shared by the Operate tab and the Test
Leg tab's gait preview so both genuinely produce the same intent from the
same keys rather than two mappings that agree until one is edited.

Qt-free on purpose (CLAUDE.md): the key *codes* are Qt's, but nothing
here imports Qt -- callers pass whatever integers their toolkit gives
them, and these tables are keyed by the Qt values as plain ints.
"""

import math

# Qt.Key_W / Key_Up etc. as plain ints -- see the module docstring for why
# this doesn't import Qt to name them.
_KEY_W, _KEY_A, _KEY_S, _KEY_D = 0x57, 0x41, 0x53, 0x44
_KEY_Q, _KEY_E = 0x51, 0x45
_KEY_LEFT, _KEY_UP, _KEY_RIGHT, _KEY_DOWN = 0x01000012, 0x01000013, 0x01000014, 0x01000015

# key -> (vx_delta, vy_delta). Diagonals sum both axes and are normalized
# to unit magnitude in walk_intent(), so diagonal walking isn't faster
# than cardinal walking.
MOVE_KEYS = {
    _KEY_W: (1.0, 0.0), _KEY_UP: (1.0, 0.0),
    _KEY_S: (-1.0, 0.0), _KEY_DOWN: (-1.0, 0.0),
    _KEY_D: (0.0, 1.0), _KEY_RIGHT: (0.0, 1.0),
    _KEY_A: (0.0, -1.0), _KEY_LEFT: (0.0, -1.0),
}
TURN_KEYS = {
    _KEY_Q: -1.0,
    _KEY_E: 1.0,
}


def walk_intent(active_move_keys) -> tuple[float, float]:
    """(vx, vy) for the currently-held movement keys, normalized to unit
    magnitude. (0, 0) when nothing is held."""
    vx = sum(MOVE_KEYS[k][0] for k in active_move_keys if k in MOVE_KEYS)
    vy = sum(MOVE_KEYS[k][1] for k in active_move_keys if k in MOVE_KEYS)
    magnitude = math.hypot(vx, vy)
    if magnitude > 1.0:
        vx, vy = vx / magnitude, vy / magnitude
    return vx, vy


def turn_intent(active_turn_keys) -> float:
    """Turn rate in [-1, 1] for the currently-held turn keys."""
    return max(-1.0, min(1.0, sum(TURN_KEYS[k] for k in active_turn_keys if k in TURN_KEYS)))
