import socket
import time

import pytest

from transport.protocol import (
    StopCommand,
    Telemetry,
    WalkCommand,
    decode_command,
    encode_telemetry,
)
from transport.udp_link import UDPRobotLink

HEARTBEAT_INTERVAL_S = 0.03
CONNECTION_TIMEOUT_S = 0.15


def make_fake_robot() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(1.0)
    return sock


@pytest.fixture
def fake_robot():
    sock = make_fake_robot()
    yield sock
    sock.close()


@pytest.fixture
def link(fake_robot):
    robot_port = fake_robot.getsockname()[1]
    link = UDPRobotLink(
        "127.0.0.1",
        robot_port,
        listen_port=0,
        heartbeat_interval_s=HEARTBEAT_INTERVAL_S,
        connection_timeout_s=CONNECTION_TIMEOUT_S,
    )
    yield link
    link.close()


def telemetry_to(link: UDPRobotLink, fake_robot: socket.socket, telemetry: Telemetry) -> None:
    my_port = link._socket.getsockname()[1]
    fake_robot.sendto(encode_telemetry(telemetry), ("127.0.0.1", my_port))


def test_default_heartbeat_is_stop_before_any_send(link, fake_robot):
    data, _addr = fake_robot.recvfrom(4096)
    received = decode_command(data)
    assert isinstance(received.command, StopCommand)


def test_send_returns_incrementing_sequence(link, fake_robot):
    fake_robot.recvfrom(4096)  # discard the first heartbeat
    seq_a = link.send(WalkCommand(vx=0.1, vy=0, speed=10))
    seq_b = link.send(WalkCommand(vx=0.2, vy=0, speed=10))
    assert seq_b > seq_a


def test_sent_command_appears_in_heartbeat_stream(link, fake_robot):
    seq = link.send(WalkCommand(vx=0.5, vy=-0.2, speed=25))
    seen = False
    for _ in range(10):
        data, _addr = fake_robot.recvfrom(4096)
        received = decode_command(data)
        if isinstance(received.command, WalkCommand) and received.seq == seq:
            seen = True
            break
    assert seen


def test_resend_keeps_arriving_while_idle(link, fake_robot):
    link.send(WalkCommand(vx=0.3, vy=0, speed=15))
    fake_robot.recvfrom(4096)
    # A second resend of the same intent should show up without another send() call.
    data, _addr = fake_robot.recvfrom(4096)
    received = decode_command(data)
    assert isinstance(received.command, WalkCommand)
    assert received.command.vx == 0.3


def test_telemetry_is_picked_up(link, fake_robot):
    seq = link.send(WalkCommand(vx=0, vy=0, speed=0))
    telemetry = Telemetry(
        seq_echo=seq,
        ok=True,
        error=None,
        fault_flags=0,
        gait_phase=0.2,
        rail_mv=None,
        link_timeout_s=1.0,
        calibration_armed=False,
        bench_armed=False,
        last_applied=None,
        profiles=None,
    )
    telemetry_to(link, fake_robot, telemetry)

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if link.latest_telemetry() is not None:
            break
        time.sleep(0.01)

    assert link.latest_telemetry() == telemetry


def test_is_connected_false_before_any_telemetry(link):
    assert link.is_connected is False


def test_is_connected_becomes_true_then_expires(link, fake_robot):
    seq = link.send(WalkCommand(vx=0, vy=0, speed=0))
    telemetry_to(
        link,
        fake_robot,
        Telemetry(
            seq_echo=seq, ok=True, error=None, fault_flags=0, gait_phase=None,
            rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
            last_applied=None, profiles=None,
        ),
    )

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and not link.is_connected:
        time.sleep(0.01)
    assert link.is_connected is True

    time.sleep(CONNECTION_TIMEOUT_S + 0.1)
    assert link.is_connected is False


def test_out_of_order_telemetry_does_not_overwrite_newer(link, fake_robot):
    newer = Telemetry(
        seq_echo=10, ok=True, error=None, fault_flags=0, gait_phase=None,
        rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
        last_applied=None, profiles=None,
    )
    older = Telemetry(
        seq_echo=3, ok=False, error="stale", fault_flags=0, gait_phase=None,
        rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
        last_applied=None, profiles=None,
    )
    telemetry_to(link, fake_robot, newer)
    time.sleep(0.1)
    telemetry_to(link, fake_robot, older)
    time.sleep(0.1)

    assert link.latest_telemetry().seq_echo == 10


def test_out_of_order_telemetry_still_counts_as_liveness(link, fake_robot):
    newer = Telemetry(
        seq_echo=10, ok=True, error=None, fault_flags=0, gait_phase=None,
        rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
        last_applied=None, profiles=None,
    )
    telemetry_to(link, fake_robot, newer)
    time.sleep(0.1)
    assert link.is_connected is True

    older = Telemetry(
        seq_echo=3, ok=True, error=None, fault_flags=0, gait_phase=None,
        rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
        last_applied=None, profiles=None,
    )
    # A stale/reordered packet still proves the link is alive right now,
    # even though its content is discarded.
    telemetry_to(link, fake_robot, older)
    time.sleep(0.1)
    assert link.is_connected is True
    assert link.latest_telemetry().seq_echo == 10


def test_malformed_telemetry_is_dropped_without_crashing(link, fake_robot):
    my_port = link._socket.getsockname()[1]
    fake_robot.sendto(b"not valid json", ("127.0.0.1", my_port))

    seq = link.send(WalkCommand(vx=0, vy=0, speed=0))
    telemetry_to(
        link,
        fake_robot,
        Telemetry(
            seq_echo=seq, ok=True, error=None, fault_flags=0, gait_phase=None,
            rail_mv=None, link_timeout_s=1.0, calibration_armed=False, bench_armed=False,
            last_applied=None, profiles=None,
        ),
    )

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        telemetry = link.latest_telemetry()
        if telemetry is not None and telemetry.seq_echo == seq:
            break
        time.sleep(0.01)

    assert link.latest_telemetry().seq_echo == seq


def test_close_stops_heartbeat(link, fake_robot):
    link.close()
    # Drain whatever was already in flight.
    fake_robot.settimeout(0.2)
    try:
        while True:
            fake_robot.recvfrom(4096)
    except socket.timeout:
        pass

    with pytest.raises(socket.timeout):
        fake_robot.recvfrom(4096)


def test_constants_warning_none_when_matching(link, fake_robot):
    from transport.generated_constants import LINK_TIMEOUT_S

    seq = link.send(WalkCommand(vx=0, vy=0, speed=0))
    telemetry_to(
        link,
        fake_robot,
        Telemetry(
            seq_echo=seq, ok=True, error=None, fault_flags=0, gait_phase=None,
            rail_mv=None, link_timeout_s=LINK_TIMEOUT_S, calibration_armed=False, bench_armed=False,
            last_applied=None, profiles=None,
        ),
    )
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and link.latest_telemetry() is None:
        time.sleep(0.01)

    assert link.constants_warning is None


def test_constants_warning_set_on_mismatch(link, fake_robot):
    seq = link.send(WalkCommand(vx=0, vy=0, speed=0))
    telemetry_to(
        link,
        fake_robot,
        Telemetry(
            seq_echo=seq, ok=True, error=None, fault_flags=0, gait_phase=None,
            rail_mv=None, link_timeout_s=999.0, calibration_armed=False, bench_armed=False,
            last_applied=None, profiles=None,
        ),
    )
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and link.latest_telemetry() is None:
        time.sleep(0.01)

    assert link.constants_warning is not None
    assert "999" in link.constants_warning
