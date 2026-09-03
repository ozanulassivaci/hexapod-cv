"""Mirrors firmware/test/test_link_watchdog/test_link_watchdog.cpp -- the
two implementations are kept in lockstep by hand (reference/ANALYSIS.md
Section 7), so every case here should exist there and vice versa.

Several of these would have failed before the re-baseline fix, and none
of them could have been written at all before transport/link_watchdog.py
existed: the mocks modelled no freshness rule whatsoever, so the entire
receive-side sequence behaviour was untested on the PC side.
"""

from transport.generated_constants import SEQUENCE_MODULUS
from transport.link_watchdog import LinkWatchdog, Observation


def test_first_packet_is_accepted_whatever_its_sequence():
    wd = LinkWatchdog(timeout_s=1.0)
    assert wd.observe_packet(12345, 0.0) is Observation.ACCEPTED
    assert wd.last_accepted_seq == 12345


def test_last_accepted_seq_is_none_before_any_packet():
    # None, not 0 -- 0 is a real sequence number and can't double as
    # "nothing yet", which is the whole reason the field is nullable.
    wd = LinkWatchdog(timeout_s=1.0)
    assert wd.last_accepted_seq is None
    assert wd.has_ever_received_packet is False


def test_newer_sequence_within_an_active_stream_is_accepted():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    assert wd.observe_packet(11, 0.1) is Observation.ACCEPTED


def test_older_sequence_within_an_active_stream_is_stale():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    assert wd.observe_packet(9, 0.1) is Observation.STALE
    # Rejected packets must not move the baseline, or one reordered
    # packet would drag the stream backwards.
    assert wd.last_accepted_seq == 10


def test_duplicate_sequence_is_stale():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    assert wd.observe_packet(10, 0.1) is Observation.STALE


def test_stale_packet_does_not_reset_the_timeout():
    # The failsafe must not be held open by packets the receiver is
    # refusing to act on.
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    wd.observe_packet(9, 0.5)
    assert wd.has_timed_out(1.1) is True


def test_stale_drops_are_counted():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    assert wd.stale_drop_count == 0
    wd.observe_packet(9, 0.1)
    wd.observe_packet(10, 0.2)
    assert wd.stale_drop_count == 2


# --- the re-baseline rule ------------------------------------------------


def test_restarted_client_is_rejected_while_the_stream_is_still_live():
    # The unchanged half of the rule: within an active stream, a peer
    # that restarts its counter is still refused. This is the reorder
    # protection docs/protocol.md Section 4 actually exists for.
    wd = LinkWatchdog(timeout_s=1.0)
    for seq in range(600):
        wd.observe_packet(seq, seq * 0.1)
    assert wd.observe_packet(0, 59.95) is Observation.STALE


def test_restarted_client_is_accepted_after_the_link_times_out():
    # The fix. A GUI restarted without power-cycling the robot begins at
    # seq 0, which is "older" than the 600 the previous session reached.
    # Before this rule it was rejected indefinitely -- a full minute of
    # dead link at the 10Hz heartbeat, silently.
    wd = LinkWatchdog(timeout_s=1.0)
    for seq in range(600):
        wd.observe_packet(seq, seq * 0.1)
    assert wd.observe_packet(0, 61.0) is Observation.NEW_SESSION
    assert wd.last_accepted_seq == 0


def test_a_new_session_does_not_count_as_a_drop():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(600, 0.0)
    wd.observe_packet(0, 5.0)
    assert wd.stale_drop_count == 0


def test_the_stream_continues_normally_after_a_re_baseline():
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(600, 0.0)
    assert wd.observe_packet(0, 5.0) is Observation.NEW_SESSION
    assert wd.observe_packet(1, 5.1) is Observation.ACCEPTED
    assert wd.observe_packet(0, 5.2) is Observation.STALE  # reorder protection is back on


def test_a_straggler_from_the_old_session_can_still_jump_the_baseline():
    # Known and accepted limitation, pinned here so it's a decision
    # rather than a surprise. After re-baselining low, a delayed packet
    # from the *previous* session still reads as "newer" by the modular
    # rule (599 is 598 ahead of 1, not behind it), so it moves the
    # baseline forward and the new session is stale again until it
    # climbs past -- the original bug, but now bounded: one more timeout
    # of silence re-baselines again.
    #
    # Not worth fixing here. It needs a packet delayed longer than the
    # whole link timeout, which a LAN does not do, and the real fix is a
    # session id in the wire format (docs/protocol.md Section 4) rather
    # than more cleverness in a comparison that only sees one number.
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(600, 0.0)
    wd.observe_packet(0, 5.0)
    wd.observe_packet(1, 5.1)
    assert wd.observe_packet(599, 5.2) is Observation.ACCEPTED
    assert wd.observe_packet(2, 5.3) is Observation.STALE
    assert wd.observe_packet(2, 7.0) is Observation.NEW_SESSION  # recovers on the next timeout


def test_re_baseline_triggers_at_exactly_the_timeout_boundary():
    # has_timed_out uses >=, and observe_packet is defined in terms of
    # has_timed_out precisely so the two can never disagree about where
    # the boundary is.
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(10, 0.0)
    assert wd.has_timed_out(1.0) is True
    assert wd.observe_packet(5, 1.0) is Observation.NEW_SESSION


def test_wraparound_is_still_handled_after_the_fix():
    # Sequence space wraps; a packet just past the wrap point is newer
    # than one just before it, and this must survive the re-baseline
    # branch being added above it.
    wd = LinkWatchdog(timeout_s=1.0)
    wd.observe_packet(SEQUENCE_MODULUS - 1, 0.0)
    assert wd.observe_packet(0, 0.1) is Observation.ACCEPTED
    assert wd.observe_packet(SEQUENCE_MODULUS - 2, 0.2) is Observation.STALE
