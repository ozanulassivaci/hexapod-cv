"""LinkWatchdog: the receive-side sequence-freshness rule, mirroring
firmware/lib/core/LinkWatchdog.{h,cpp} exactly -- the same "two
independent implementations kept in lockstep" pattern this project uses
for kinematics and gait (reference/ANALYSIS.md Section 7), applied to the
one piece of receiver logic the mocks previously didn't model at all.

That gap was not theoretical. MockRobotLink and SimRobotLink accepted
every packet unconditionally, so a firmware bug in exactly this rule --
a restarted PC's sequence counter being rejected indefinitely, leaving
the GUI's LINK light red with no log line anywhere -- was invisible to
the entire test suite and to every mock-mode session. It took a
standalone diagnostic tool and a firmware source read to find. This
module exists so the next one fails a test instead.

The timeout half of firmware's LinkWatchdog (hasTimedOut driving safe
state) has no counterpart here: RobotLink.is_connected already owns
"have we heard from the robot recently" on the PC side. What this class
needs the timeout for is the re-baseline rule below, not failsafe.
"""

from enum import Enum, auto

from transport.generated_constants import LINK_TIMEOUT_S
from transport.protocol import sequence_is_newer


class Observation(Enum):
    """Outcome of one inbound packet. Three states, not a bool: a
    re-baseline is a real state change worth logging and counting, and
    collapsing it into "accepted" is what made the original bug's
    recovery path unobservable."""

    ACCEPTED = auto()
    STALE = auto()
    NEW_SESSION = auto()


class LinkWatchdog:
    def __init__(self, timeout_s: float = LINK_TIMEOUT_S) -> None:
        self._timeout_s = timeout_s
        self._has_received = False
        self._last_seq = 0
        self._last_seen_at_s = 0.0
        self.stale_drop_count = 0

    @property
    def has_ever_received_packet(self) -> bool:
        return self._has_received

    @property
    def last_accepted_seq(self) -> int | None:
        """None until a packet has actually been accepted -- 0 would be a
        real sequence number, so it can't double as "nothing yet"."""
        return self._last_seq if self._has_received else None

    def has_timed_out(self, now_s: float) -> bool:
        if not self._has_received:
            return True
        return (now_s - self._last_seen_at_s) >= self._timeout_s

    def seconds_since_last_packet(self, now_s: float) -> float:
        if not self._has_received:
            return -1.0
        return now_s - self._last_seen_at_s

    def observe_packet(self, seq: int, now_s: float) -> Observation:
        """Latest-wins freshness, with one exception: if the link has
        already been silent longer than the timeout, the next packet is
        accepted regardless of its sequence number and becomes the new
        baseline.

        The exception exists because the plain rule assumed one PC
        counter for the life of the robot's boot. Restart the GUI (or
        run a diagnostic tool) without power-cycling the robot and the
        new process starts at seq 0, which is "older" than whatever the
        previous session reached -- so every packet it will ever send is
        rejected, silently, until its counter climbs past the old one.
        At a 10Hz heartbeat and a one-minute previous session that is a
        full minute of dead link; longer sessions, proportionally worse.

        Re-baselining is safe precisely where it triggers: the link is
        already past its timeout, which means the failsafe has already
        fired and the robot is already in safe state. A peer arriving
        after that is a new session by definition, and there is nothing
        left for the stale check to protect. Inside an active stream --
        the case the rule actually exists for -- reordered and duplicate
        packets are rejected exactly as before.

        The timeout comparison is has_timed_out() itself, not a
        re-derived expression, so "the watchdog considers the link down"
        and "the next packet re-baselines" cannot drift apart.
        """
        result = Observation.ACCEPTED
        if self._has_received:
            if self.has_timed_out(now_s):
                result = Observation.NEW_SESSION
            elif not sequence_is_newer(seq, self._last_seq):
                self.stale_drop_count += 1
                return Observation.STALE

        self._last_seq = seq
        self._last_seen_at_s = now_s
        self._has_received = True
        return result
