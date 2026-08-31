"""RobotLink: transport-agnostic interface for sending commands to, and
receiving telemetry from, the robot. A concrete link (UDPRobotLink,
MockRobotLink) must always be usable with no hardware attached -- see
mock_link.py.

Shared bookkeeping (latest telemetry, connection liveness, the
timeout-mismatch cross-check, out-of-order telemetry rejection) lives here
so every concrete link gets it for free; only send() and close() are
transport-specific.
"""

import time
from abc import ABC, abstractmethod
from threading import Lock

from transport.generated_constants import LINK_TIMEOUT_S
from transport.protocol import Command, Telemetry, sequence_is_newer

_TIMEOUT_TOLERANCE_S = 0.01


class RobotLink(ABC):
    def __init__(self, connection_timeout_s: float = LINK_TIMEOUT_S) -> None:
        self._connection_timeout_s = connection_timeout_s
        self._lock = Lock()
        self._latest_telemetry: Telemetry | None = None
        self._latest_telemetry_at: float | None = None

    @abstractmethod
    def send(self, command: Command) -> int:
        """Encode and transmit command. Returns the sequence number it was
        sent with, so a caller can correlate a later Telemetry.seq_echo --
        see wait_for_ack/send_and_wait below."""
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    def latest_telemetry(self) -> Telemetry | None:
        with self._lock:
            return self._latest_telemetry

    @property
    def is_connected(self) -> bool:
        """Whether telemetry (any of it, not necessarily the newest) has
        been received recently -- reflects "have we heard from the robot",
        not just "is the socket open"."""
        with self._lock:
            received_at = self._latest_telemetry_at
        if received_at is None:
            return False
        return (time.monotonic() - received_at) <= self._connection_timeout_s

    @property
    def constants_warning(self) -> str | None:
        """Non-None if the most recent telemetry's link_timeout_s disagrees
        with this process's own LINK_TIMEOUT_S. This is the runtime
        cross-check that stands in for build-time codegen enforcement --
        see docs/protocol.md Section 5."""
        telemetry = self.latest_telemetry()
        if telemetry is None:
            return None
        if abs(telemetry.link_timeout_s - LINK_TIMEOUT_S) > _TIMEOUT_TOLERANCE_S:
            return (
                f"link timeout mismatch: local={LINK_TIMEOUT_S}s, "
                f"robot reports {telemetry.link_timeout_s}s"
            )
        return None

    def wait_for_ack(
        self, seq: int, timeout_s: float = 2.0, poll_interval_s: float = 0.02
    ) -> Telemetry | None:
        """Block until telemetry echoing `seq` arrives, or timeout_s elapses.
        Motion commands don't need this -- they free-run and get superseded
        by the next resend regardless. Calibration-family commands do: the
        explicit ack round-trip is what lets a caller confirm one write
        landed before sending the next."""
        deadline = time.monotonic() + timeout_s
        while True:
            telemetry = self.latest_telemetry()
            if telemetry is not None and telemetry.seq_echo == seq:
                return telemetry
            if time.monotonic() >= deadline:
                return None
            time.sleep(poll_interval_s)

    def send_and_wait(self, command: Command, timeout_s: float = 2.0) -> Telemetry | None:
        seq = self.send(command)
        return self.wait_for_ack(seq, timeout_s=timeout_s)

    def _record_telemetry(self, telemetry: Telemetry) -> bool:
        """Call this whenever a telemetry packet is actually decoded off a
        real transport (currently: UDPRobotLink._receive_loop only). Any
        receipt counts as proof of life for is_connected; only a *newer*
        seq_echo replaces the stored telemetry content, same latest-wins
        rule commands use -- a reordered, stale, or duplicate wire packet
        must not un-show a more recent one. Returns whether the content
        was accepted.

        There is exactly one caller left, on purpose: MockRobotLink and
        SimRobotLink compute telemetry locally, with no real packet or
        network behind it -- "did a newer packet arrive" is a category
        error there, not a rare edge case, since nothing can reorder or
        duplicate a single in-process computation. Route a
        locally-synthesized caller to _record_local_telemetry() below
        instead of adding another opt-out parameter here; this method's
        contract stays narrow (real wire packets only) on purpose."""
        now = time.monotonic()
        with self._lock:
            self._latest_telemetry_at = now
            if self._latest_telemetry is not None:
                if not sequence_is_newer(telemetry.seq_echo, self._latest_telemetry.seq_echo):
                    return False
            self._latest_telemetry = telemetry
            return True

    def _record_local_telemetry(self, telemetry: Telemetry) -> None:
        """For telemetry synthesized in-process -- every MockRobotLink and
        SimRobotLink call, with no real packet or transport underneath
        any of them. Always accepts and marks a fresh is_connected
        timestamp, same as _record_telemetry's proof-of-life behavior,
        but with no seq_echo comparison at all: there is nothing here a
        network could have reordered or duplicated, only one in-process
        computation of "what's true right now," so the newer-seq-only
        rule that method exists to enforce is not a weaker version of
        what's needed here, it's simply not the applicable check."""
        with self._lock:
            self._latest_telemetry_at = time.monotonic()
            self._latest_telemetry = telemetry

    def __enter__(self) -> "RobotLink":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
