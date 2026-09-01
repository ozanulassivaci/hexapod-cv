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


class MomentaryHold:
    """Tracks a deliberately brief hold at a convenience extreme (the
    Bench Test tab's 0/180-equivalent buttons), auto-returning to
    neutral after hold_s regardless of DwellGuard's own, longer,
    away-from-neutral timeout -- these buttons drive toward an extreme
    on purpose, so the short, unconditional window is what keeps a
    clone servo that can't actually reach the nominal extreme from
    being left fighting it."""

    def __init__(self, hold_s: float = 2.0) -> None:
        self.hold_s = hold_s
        self._started_at: float | None = None

    @property
    def pending(self) -> bool:
        return self._started_at is not None

    def start(self, now: float) -> None:
        self._started_at = now

    def cancel(self) -> None:
        self._started_at = None

    def remaining_s(self, now: float) -> float | None:
        if self._started_at is None:
            return None
        return max(0.0, self.hold_s - (now - self._started_at))

    def observe(self, now: float) -> bool:
        """Call every tick while pending. Returns True exactly once, the
        moment the hold expires (and clears pending state) -- the
        caller's cue to send the return-to-neutral pulse."""
        if self._started_at is None:
            return False
        if now - self._started_at < self.hold_s:
            return False
        self._started_at = None
        return True


class RepeatabilityCheck:
    """Drives neutral -> zero -> neutral (a convenience extreme, not the
    servo's true 0), holding step_hold_s at each stop so the operator has
    time to look. There is no position feedback anywhere in this system
    (CLAUDE.md) -- "did it return to the same point" can only ever be a
    human's own visual judgment; this class only paces a hands-free
    sequence for them to watch, never decides the answer itself."""

    STEPS = ("neutral", "zero", "neutral")

    def __init__(self, step_hold_s: float = 2.0) -> None:
        self.step_hold_s = step_hold_s
        self._index: int | None = None
        self._step_started_at: float | None = None

    @property
    def active(self) -> bool:
        return self._index is not None

    @property
    def current_target(self) -> str | None:
        return self.STEPS[self._index] if self.active else None

    def start(self, now: float) -> None:
        self._index = 0
        self._step_started_at = now

    def cancel(self) -> None:
        self._index = None
        self._step_started_at = None

    def tick(self, now: float) -> str | None:
        """Call every tick while active. Returns the new current_target
        the instant the sequence advances to it (the caller's cue to
        send that pulse), "done" once every step has held its full
        duration, or None if nothing changed yet."""
        if not self.active:
            return None
        if now - self._step_started_at < self.step_hold_s:
            return None
        self._index += 1
        if self._index >= len(self.STEPS):
            self._index = None
            self._step_started_at = None
            return "done"
        self._step_started_at = now
        return self.current_target


class HoldCheck:
    """Paces a park-and-listen wait (default 30s) so the operator can
    check for hunting/buzzing -- a stalled or miscalibrated servo
    audibly fights to hold position even when commanded to sit still.
    Whether it did is the operator's own judgment; this only times the
    wait, the same "never decides the answer" split as
    RepeatabilityCheck."""

    def __init__(self, hold_s: float = 30.0) -> None:
        self.hold_s = hold_s
        self._started_at: float | None = None

    @property
    def active(self) -> bool:
        return self._started_at is not None

    def start(self, now: float) -> None:
        self._started_at = now

    def cancel(self) -> None:
        self._started_at = None

    def remaining_s(self, now: float) -> float | None:
        if self._started_at is None:
            return None
        return max(0.0, self.hold_s - (now - self._started_at))

    def tick(self, now: float) -> bool:
        """Call every tick while active. Returns True exactly once, the
        moment the hold completes (and clears active state)."""
        if self._started_at is None:
            return False
        if now - self._started_at < self.hold_s:
            return False
        self._started_at = None
        return True
