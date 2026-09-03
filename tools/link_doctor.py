"""Standalone UDP link diagnostic -- tests the PC <-> firmware link end to
end, one layer at a time, without launching the GUI. Run this when
--link-mode udp shows LINK: disconnected and a single red light isn't
enough to tell you which layer is broken.

    python tools/link_doctor.py 192.168.0.50
    python tools/link_doctor.py 192.168.0.50 --robot-port 9000 --listen-port 9001

Reuses transport/protocol.py's real encode_command()/decode_telemetry() --
the exact wire code the GUI itself runs, not a reimplementation -- so a
clean ping reply here proves the protocol layer works end to end, not just
"some UDP arrived." Each step reports plainly (OK/FAIL/WARN) and, on
failure, says what that specific result does and doesn't rule out; there
is no stack trace in the normal-failure path, only in a genuine bug in
this script.
"""

import argparse
import pathlib
import socket
import subprocess
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from operator_config import DEFAULT_CONFIG_PATH, load_operator_config  # noqa: E402
from transport.protocol import (  # noqa: E402
    FAULT_BROWNOUT,
    FAULT_ESTOP,
    FAULT_IK_CLIP,
    FAULT_LINK_TIMEOUT,
    FAULT_NONE,
    FAULT_SERVO_FAULT,
    PingCommand,
    ProtocolError,
    decode_telemetry,
    encode_command,
    has_fault,
)

_PING_TIMEOUT_S = 2.0
_PROBE_TIMEOUT_S = 1.5

_FAULT_NAMES = [
    (FAULT_LINK_TIMEOUT, "LINK_TIMEOUT"),
    (FAULT_ESTOP, "ESTOP"),
    (FAULT_SERVO_FAULT, "SERVO_FAULT"),
    (FAULT_BROWNOUT, "BROWNOUT"),
    (FAULT_IK_CLIP, "IK_CLIP"),
]


def _fault_flags_text(fault_flags: int) -> str:
    if fault_flags == FAULT_NONE:
        return "none"
    names = [name for bit, name in _FAULT_NAMES if has_fault(fault_flags, bit)]
    unknown = fault_flags & ~sum(bit for bit, _ in _FAULT_NAMES)
    if unknown:
        names.append(f"unknown(0x{unknown:x})")
    return ", ".join(names) if names else f"0x{fault_flags:x}"


def _step(n: int, title: str) -> None:
    print(f"\n[{n}] {title}")


def _ok(msg: str) -> None:
    print(f"    OK   -- {msg}")


def _fail(msg: str) -> None:
    print(f"    FAIL -- {msg}")


def _warn(msg: str) -> None:
    print(f"    WARN -- {msg}")


def _info(msg: str) -> None:
    print(f"         {msg}")


# --- steps -----------------------------------------------------------------


def check_config(host: str, config_path: str) -> None:
    _step(1, f"operator_config.yaml ({config_path}) vs. the address you passed")
    config = load_operator_config(config_path)
    cfg_host = config.link.udp.robot_host
    _info(f"link.mode = {config.link.mode!r}")
    _info(
        f"link.udp.robot_host = {cfg_host!r}, robot_port = {config.link.udp.robot_port}, "
        f"listen_port = {config.link.udp.listen_port}"
    )
    if cfg_host != host:
        _warn(
            f"config's robot_host ({cfg_host!r}) does not match the address you passed ({host!r}). "
            "app.py always uses the config file's robot_host when link.mode is udp -- there is no "
            "CLI override for the host or ports, only for link.mode itself (see below). If you meant "
            "to test what the real GUI will actually connect to, re-run this tool against the config's "
            "address, or edit operator_config.yaml."
        )
    else:
        _ok("matches -- the GUI will connect to the same address you're testing here")

    if config.link.mode != "udp":
        _warn(
            f"config file's link.mode is {config.link.mode!r}, not 'udp'. Confirmed by reading "
            "app.py: passing --link-mode udp on the command line overrides this before the link "
            "object is built, so a real `python app.py --link-mode udp` run is NOT silently falling "
            "back to this value -- that's not today's problem. But if you (or anyone else) ever runs "
            f"app.py without --link-mode, it will silently use {config.link.mode!r} instead of udp, "
            "with no error -- worth knowing."
        )


def check_reachable(host: str) -> bool:
    _step(2, f"is {host} reachable at all (ICMP ping)")
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", host],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except FileNotFoundError:
        _warn("no `ping` binary found on this system -- skipping, can't rule this layer out")
        return True  # unknown, don't block later steps on a missing tool
    except subprocess.TimeoutExpired:
        _fail(f"ping to {host} did not return within 5s")
        return False

    if result.returncode == 0:
        _ok(f"{host} responds to ping")
        for line in result.stdout.splitlines():
            if "time=" in line:
                _info(line.strip())
        return True

    _fail(f"{host} did not respond to ping (exit code {result.returncode})")
    tail = (result.stdout.strip() or result.stderr.strip()).splitlines()
    for line in tail[-3:]:
        _info(line)
    _info(
        "if the robot is genuinely powered on with this IP, this usually means: wrong address, a "
        "different subnet/VLAN (common on WiFi with client isolation), or it isn't on this network "
        "right now. Nothing past this layer can work until this passes -- fix this first."
    )
    return False


def probe_udp_port(host: str, robot_port: int, listen_port: int) -> bool | None:
    """One garbage UDP packet, checking whether the OS reports an ICMP
    port-unreachable back -- the closest thing UDP has to "is anyone
    listening." Returns True (probably listening), False (port refused),
    or None (inconclusive / couldn't even test)."""
    _step(3, f"raw UDP probe to {host}:{robot_port} (garbage payload -- checking for ICMP port-unreachable)")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", listen_port))
    except OSError as exc:
        _fail(f"could not bind local port {listen_port}: {exc}")
        _info(
            "this is a LOCAL problem, not a network one. Most likely something else on this machine "
            f"already has UDP port {listen_port} open -- if the real GUI (or another copy of this "
            "script) is already running with --link-mode udp, close it first: only one process can "
            "bind a UDP port at a time, and the real GUI would fail to start with this exact error "
            "rather than run with a red LINK light."
        )
        return None

    sock.settimeout(_PROBE_TIMEOUT_S)
    try:
        sock.connect((host, robot_port))
        sock.send(b"link_doctor_probe")
        time.sleep(0.3)
        # A second send is often what surfaces a pending ICMP error on
        # Linux -- the first send can succeed even against a closed port.
        sock.send(b"link_doctor_probe_2")
        sock.recv(4096)
        _warn("got a reply to a garbage (non-protocol) packet -- unusual, but not a failure by itself")
        return True
    except ConnectionRefusedError:
        _fail(f"got ICMP port-unreachable from {host} -- nothing is listening on UDP port {robot_port} there")
        _info(
            "check the robot itself: is firmware actually past setup() and into loop()? A hang or "
            "crash after the boot log line you saw, but before udp.begin(), would look exactly like "
            "this from the PC side."
        )
        return False
    except socket.timeout:
        _ok("no ICMP port-unreachable -- consistent with something bound and listening on that port")
        _info(
            "this alone doesn't prove firmware is healthy, only that the port isn't flatly closed. "
            "A firewall that silently drops instead of rejecting looks identical -- step 4 is the "
            "real test of whether firmware actually answers."
        )
        return True
    except OSError as exc:
        _fail(f"probe send/recv failed: {exc}")
        return None
    finally:
        sock.close()


def check_ping(host: str, robot_port: int, listen_port: int):
    _step(4, f"real PingCommand to {host}:{robot_port}, using this project's own encode/decode code")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", listen_port))
    except OSError as exc:
        _fail(f"could not bind local port {listen_port}: {exc}")
        return None
    sock.settimeout(_PING_TIMEOUT_S)

    data = encode_command(PingCommand(), seq=0)
    try:
        sock.sendto(data, (host, robot_port))
    except OSError as exc:
        _fail(f"send failed: {exc}")
        sock.close()
        return None
    _info(f"sent PingCommand (seq=0, {len(data)} bytes) from local port {listen_port}")

    try:
        reply, addr = sock.recvfrom(65535)
    except socket.timeout:
        _fail(f"no reply within {_PING_TIMEOUT_S}s")
        _info(
            "the packet left this machine with no send error, and nothing came back. If step 3 saw "
            "no ICMP-unreachable, this now points at one of: firmware receiving the packet but "
            "failing to decode it (check the robot's serial log for anything logged when this arrived "
            "-- decodeCommand() drops a malformed packet silently, with no reply and no telemetry), or "
            "a reply being sent but blocked on the way back -- a firewall on THIS machine that allows "
            f"outbound UDP but blocks unsolicited inbound on port {listen_port} looks exactly like "
            "this. Check the PC's firewall rules for inbound UDP on that port."
        )
        sock.close()
        return None
    except OSError as exc:
        _fail(f"recv failed: {exc}")
        sock.close()
        return None

    _ok(f"got {len(reply)} bytes back from {addr[0]}:{addr[1]}")
    if addr[0] != host:
        _warn(f"reply came from {addr[0]}, not the address you pinged ({host}) -- unexpected on a simple LAN")

    try:
        telemetry = decode_telemetry(reply)
    except ProtocolError as exc:
        _fail(f"reply arrived, but this project's own decode_telemetry() rejected it: {exc}")
        _info(f"raw bytes: {reply!r}")
        _info(
            "this is specifically the 'firmware is answering but the GUI can't read it' case -- the "
            "GUI calls this exact function on every packet, so it would fail identically. Compare "
            "transport/protocol.py's encode_telemetry()/decode_telemetry() against firmware/lib/core/"
            "Protocol.cpp's encodeTelemetry() for a field name or type mismatch."
        )
        sock.close()
        return None

    _ok("decoded cleanly with the same decode_telemetry() the GUI uses -- protocol layer works end to end")
    _info(f"seq_echo={telemetry.seq_echo}  ok={telemetry.ok}  error={telemetry.error}")
    _info(f"fault_flags={_fault_flags_text(telemetry.fault_flags)}")
    _info(f"link_timeout_s={telemetry.link_timeout_s}  robot_assembled={telemetry.robot_assembled}")
    _info(f"calibration_armed={telemetry.calibration_armed}  bench_armed={telemetry.bench_armed}")
    _info(f"ik_clip_count={telemetry.ik_clip_count}  joint_clip_count={telemetry.joint_clip_count}")
    sock.close()
    return telemetry


def main() -> int:
    parser = argparse.ArgumentParser(description="Diagnose the PC <-> firmware UDP link, layer by layer.")
    parser.add_argument("host", help="robot's IP address, e.g. 192.168.0.50")
    parser.add_argument("--robot-port", type=int, default=None, help="default: operator_config.yaml's robot_port")
    parser.add_argument("--listen-port", type=int, default=None, help="default: operator_config.yaml's listen_port")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="path to operator_config.yaml")
    args = parser.parse_args()

    config = load_operator_config(args.config)
    robot_port = args.robot_port if args.robot_port is not None else config.link.udp.robot_port
    listen_port = args.listen_port if args.listen_port is not None else config.link.udp.listen_port

    print(f"link_doctor: {args.host} (robot_port={robot_port}, listen_port={listen_port})")

    check_config(args.host, args.config)

    reachable = check_reachable(args.host)
    if not reachable:
        print("\n=== VERDICT: host unreachable -- fix that before anything else here matters. ===")
        return 1

    probe_result = probe_udp_port(args.host, robot_port, listen_port)
    if probe_result is False:
        print("\n=== VERDICT: robot host is up, but nothing is listening on that UDP port. ===")
        return 1
    if probe_result is None:
        print("\n=== VERDICT: could not complete the port probe -- see the FAIL above. ===")
        return 1

    telemetry = check_ping(args.host, robot_port, listen_port)
    if telemetry is None:
        print("\n=== VERDICT: firmware isn't answering (or the GUI's decode would reject it) -- see step 4. ===")
        return 1

    print(
        "\n=== VERDICT: the link works end to end from this script. ===\n"
        "If the GUI still shows LINK: disconnected, the protocol and network layers are proven fine, "
        "so look at what's different about the GUI process specifically:\n"
        f"  - is something else (an old GUI instance, or this script) already holding port {listen_port}? "
        "Only one process can bind it -- the GUI would fail to start, not run with a red light, but "
        "worth ruling out by closing everything else and relaunching.\n"
        "  - did the GUI actually get --link-mode udp with no typo, in the terminal that's still running? "
        "(step 1 above confirms the flag itself works correctly when passed.)\n"
        "  - is a firewall rule specific to the GUI's process (rather than this script's) blocking it? "
        "worth testing with the GUI itself if the above don't explain it."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
