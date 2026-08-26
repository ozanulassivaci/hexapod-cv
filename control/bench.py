"""Bench-test pure logic: stall/dwell protection and sweep-pulse math.

No Qt imports -- ui/bench_tab.py drives these with wall-clock time it
supplies explicitly (never reads the clock itself here), same separation
as control/tracker.py, and for the same reason: unit-testable without a
display, an event loop, or real sleeping in tests.

DwellGuard intentionally protects against ANY sustained non-neutral hold,
not just a hold at a recorded limit. The most dangerous moment for a servo
is *before* its limits are known -- that's what the range finder is for --
and protection that only activates once a limit is recorded would leave
exactly that exploratory phase unprotected. An active sweep does not go
through DwellGuard at all: it's continuously moving, not stalling, and
it's already bounded (one pass, finite duration) and abortable on its own.
"""

from dataclasses import dataclass

from transport.protocol import NEUTRAL_PULSE_US


@dataclass(frozen=True)
class BenchSafetyConfig:
    neutral_pulse_us: int = NEUTRAL_PULSE_US
    dwell_timeout_s: float = 8.0

    def __post_init__(self) -> None:
        if self.dwell_timeout_s <= 0:
            raise ValueError(f"dwell_timeout_s must be positive, got {self.dwell_timeout_s!r}")


class DwellGuard:
    """Tracks how long a non-neutral bench pulse has been held. Call
    observe() every tick with the pulse currently being commanded and the
    current time; when it returns True, the caller should force a return
    to neutral now."""

    def __init__(self, config: BenchSafetyConfig) -> None:
        self.config = config
        self._away_since: float | None = None

    def observe(self, pulse_us: int, now: float) -> bool:
        if pulse_us == self.config.neutral_pulse_us:
            self._away_since = None
            return False
        if self._away_since is None:
            self._away_since = now
            return False
        return (now - self._away_since) >= self.config.dwell_timeout_s

    def remaining_s(self, pulse_us: int, now: float) -> float | None:
        """Seconds until auto-return, or None if not currently at risk (at
        neutral, or observe() hasn't been called with a non-neutral pulse
        yet) -- for a visible "returning to neutral in Xs" warning."""
        if pulse_us == self.config.neutral_pulse_us or self._away_since is None:
            return None
        elapsed = now - self._away_since
        return max(0.0, self.config.dwell_timeout_s - elapsed)

    def reset(self) -> None:
        self._away_since = None


@dataclass(frozen=True)
class SweepPlan:
    """One pass, min_us -> max_us, linear over duration_s. The caller
    (ui/bench_tab.py) polls pulse_at() each tick with elapsed time since
    the sweep started and checks is_complete() to know when to stop -- this
    class has no concept of wall-clock time itself."""

    min_us: int
    max_us: int
    duration_s: float

    def __post_init__(self) -> None:
        if self.min_us >= self.max_us:
            raise ValueError(f"min_us ({self.min_us}) must be < max_us ({self.max_us})")
        if self.duration_s <= 0:
            raise ValueError(f"duration_s must be positive, got {self.duration_s!r}")

    def pulse_at(self, elapsed_s: float) -> int:
        t = max(0.0, min(elapsed_s, self.duration_s))
        fraction = t / self.duration_s
        return round(self.min_us + fraction * (self.max_us - self.min_us))

    def is_complete(self, elapsed_s: float) -> bool:
        return elapsed_s >= self.duration_s
