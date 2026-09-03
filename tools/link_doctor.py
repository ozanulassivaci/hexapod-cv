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

Steps 2 and 4 send several probes, not one, and report round-trip time
explicitly. A single probe can get lucky: this project's own real-hardware
link once passed roughly 2 times in 50 attempts, at 883ms RTT when it did
-- indistinguishable from "it just works" on a single try, and
indistinguishable from "it's dead" on an unlucky one. That specific
profile (mostly no reply, occasional very slow reply) is the signature of
ESP32 WiFi modem sleep -- the radio powers down between packets and takes
hundreds of ms to wake -- and is flagged explicitly, not just reported as
a number, since it's easy to read past a big RTT if the step still says
OK. See docs/HOW_TO_USE.md's link troubleshooting section.
"""

import argparse
import pathlib
import re
import socket
import subprocess
import sys
import time
from dataclasses import dataclass

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
    Telemetry,
    decode_telemetry,
    encode_command,
    has_fault,
)

_PING_TIMEOUT_S = 2.0  # per-probe wait in step 4 -- generous on purpose, see main()'s docstring note
_PROBE_TIMEOUT_S = 1.5
_PING_ATTEMPTS = 10
_ICMP_PING_COUNT = 5

# A healthy LAN round trip is roughly 1-10ms. These thresholds are
# deliberately well above normal jitter before warning, and set high
# enough at the top end that only a real modem-sleep-scale wake delay
# (hundreds of ms) trips it -- see the module docstring.
_RTT_ELEVATED_MS = 50.0
_RTT_MODEM_SLEEP_MS = 200.0

_RTT_SUMMARY_RE = re.compile(r"=\s*([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)\s*ms")
_LOSS_RE = re.compile(r"(\d+)% packet loss")

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


def _rtt_note(rtt_ms: float) -> str | None:
    if rtt_ms >= _RTT_MODEM_SLEEP_MS:
        return (
            f"{rtt_ms:.0f}ms is far past normal LAN latency (expect roughly 1-10ms) and matches the "
            "signature of ESP32 WiFi modem sleep: the radio powers down between packets and takes "
            "hundreds of ms to wake for the next one. See docs/HOW_TO_USE.md's link troubleshooting "
            "section -- the fix is esp_wifi_set_ps(WIFI_PS_NONE) in firmware, not anything on the PC side."
        )
    if rtt_ms >= _RTT_ELEVATED_MS:
        return (
            f"{rtt_ms:.0f}ms is higher than a healthy LAN should show (expect roughly 1-10ms) -- short "
            "of the modem-sleep range, but worth a second look."
        )
    return None


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


def check_reachable(host: str, count: int = _ICMP_PING_COUNT) -> bool:
    _step(2, f"is {host} reachable at all ({count}x ICMP ping)")
    try:
        result = subprocess.run(
            ["ping", "-c", str(count), "-W", "2", host],
            capture_output=True,
            text=True,
            timeout=count * 3 + 5,
        )
    except FileNotFoundError:
        _warn("no `ping` binary found on this system -- skipping, can't rule this layer out")
        return True  # unknown, don't block later steps on a missing tool
    except subprocess.TimeoutExpired:
        _fail(f"ping to {host} did not return in time")
        return False

    loss_match = _LOSS_RE.search(result.stdout)
    loss_pct = int(loss_match.group(1)) if loss_match else None
    rtt_match = _RTT_SUMMARY_RE.search(result.stdout)

    if loss_pct == 100 or rtt_match is None and result.returncode != 0:
        _fail(f"{host} did not respond to any of {count} pings")
        tail = (result.stdout.strip() or result.stderr.strip()).splitlines()
        for line in tail[-3:]:
            _info(line)
        _info(
            "if the robot is genuinely powered on with this IP, this usually means: wrong address, a "
            "different subnet/VLAN (common on WiFi with client isolation), or it isn't on this network "
            "right now. Nothing past this layer can work until this passes -- fix this first."
        )
        return False

    if rtt_match is None:
        _warn(f"{host} responded to some pings, but couldn't parse the RTT summary; raw output below")
        _info(result.stdout.strip())
        return True

    rtt_min, rtt_avg, rtt_max, _mdev = (float(x) for x in rtt_match.groups())
    summary = f"min/avg/max = {rtt_min:.0f}/{rtt_avg:.0f}/{rtt_max:.0f} ms"
    if loss_pct:
        _warn(f"{loss_pct}% packet loss over {count} pings ({summary})")
    else:
        _ok(f"{host} responds to all {count} pings ({summary})")

    note = _rtt_note(rtt_max)
    if note:
        _warn(note)
    elif loss_pct:
        _info(
            "packet loss without a high RTT doesn't match the modem-sleep signature by itself -- "
            "step 4 below is the more informative test either way."
        )
    return True


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
            "real test of whether firmware actually answers, and how reliably."
        )
        return True
    except OSError as exc:
        _fail(f"probe send/recv failed: {exc}")
        return None
    finally:
        sock.close()


@dataclass
class PingResult:
    telemetry: Telemetry | None
    successes: int
    attempts: int
    rtt_min_ms: float | None
    rtt_avg_ms: float | None
    rtt_max_ms: float | None


def check_ping(host: str, robot_port: int, listen_port: int, attempts: int = _PING_ATTEMPTS) -> PingResult | None:
    _step(4, f"{attempts}x real PingCommand to {host}:{robot_port}, using this project's own encode/decode code")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", listen_port))
    except OSError as exc:
        _fail(f"could not bind local port {listen_port}: {exc}")
        return None
    sock.settimeout(_PING_TIMEOUT_S)

    rtts_ms: list[float] = []
    last_telemetry: Telemetry | None = None
    decode_error: str | None = None
    last_reply: bytes | None = None
    for seq in range(attempts):
        data = encode_command(PingCommand(), seq=seq)
        sent_at = time.monotonic()
        try:
            sock.sendto(data, (host, robot_port))
            reply, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError as exc:
            _fail(f"send/recv failed on attempt {seq}: {exc}")
            sock.close()
            return None
        rtts_ms.append((time.monotonic() - sent_at) * 1000.0)
        try:
            last_telemetry = decode_telemetry(reply)
        except ProtocolError as exc:
            decode_error = str(exc)
            last_reply = reply
            break
        time.sleep(0.05)  # a small gap between probes, not a tight flood
    sock.close()

    successes = len(rtts_ms)
    _info(f"{successes}/{attempts} replies received")

    if decode_error is not None:
        _fail(f"a reply arrived, but this project's own decode_telemetry() rejected it: {decode_error}")
        _info(f"raw bytes: {last_reply!r}")
        _info(
            "this is specifically the 'firmware is answering but the GUI can't read it' case -- the "
            "GUI calls this exact function on every packet, so it would fail identically. Compare "
            "transport/protocol.py's encode_telemetry()/decode_telemetry() against firmware/lib/core/"
            "Protocol.cpp's encodeTelemetry() for a field name or type mismatch."
        )
        return None

    if successes == 0:
        _fail(f"no reply in any of {attempts} attempts ({_PING_TIMEOUT_S}s timeout each)")
        _info(
            "every packet left this machine with no send error, and nothing came back, not even once. "
            "If step 3 saw no ICMP-unreachable, this points at: firmware receiving packets but failing "
            "to decode them (check the robot's serial log -- decodeCommand() drops a malformed packet "
            "silently, no reply, no telemetry), or replies being sent but blocked on the way back here "
            f"-- a firewall on this machine blocking unsolicited inbound UDP on port {listen_port} "
            "looks exactly like this."
        )
        return PingResult(None, 0, attempts, None, None, None)

    rtt_min, rtt_avg, rtt_max = min(rtts_ms), sum(rtts_ms) / len(rtts_ms), max(rtts_ms)
    _info(f"RTT over {successes} successful replies: min/avg/max = {rtt_min:.0f}/{rtt_avg:.0f}/{rtt_max:.0f} ms")

    note = _rtt_note(rtt_max)
    if successes < attempts and note:
        loss_pct = 100 * (attempts - successes) / attempts
        _fail(
            f"{loss_pct:.0f}% of pings got no reply at all, and the ones that did took up to "
            f"{rtt_max:.0f}ms -- this is an intermittent-high-latency pattern, not a clean pass or a "
            "clean failure."
        )
        _info(note)
    elif note:
        _warn(note)
    elif successes < attempts:
        loss_pct = 100 * (attempts - successes) / attempts
        _warn(f"{loss_pct:.0f}% of pings got no reply, even though RTT looks normal -- worth a second run")
    else:
        _ok("decoded cleanly every time, with the same decode_telemetry() the GUI uses -- protocol layer works end to end")

    t = last_telemetry
    _info(f"seq_echo={t.seq_echo}  ok={t.ok}  error={t.error}")
    _info(f"fault_flags={_fault_flags_text(t.fault_flags)}")
    _info(f"link_timeout_s={t.link_timeout_s}  robot_assembled={t.robot_assembled}")
    _info(f"calibration_armed={t.calibration_armed}  bench_armed={t.bench_armed}")
    _info(f"ik_clip_count={t.ik_clip_count}  joint_clip_count={t.joint_clip_count}")
    return PingResult(last_telemetry, successes, attempts, rtt_min, rtt_avg, rtt_max)


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

    if not check_reachable(args.host):
        print("\n=== VERDICT: host unreachable -- fix that before anything else here matters. ===")
        return 1

    probe_result = probe_udp_port(args.host, robot_port, listen_port)
    if probe_result is False:
        print("\n=== VERDICT: robot host is up, but nothing is listening on that UDP port. ===")
        return 1
    if probe_result is None:
        print("\n=== VERDICT: could not complete the port probe -- see the FAIL above. ===")
        return 1

    result = check_ping(args.host, robot_port, listen_port)
    if result is None or result.telemetry is None:
        print("\n=== VERDICT: firmware isn't answering (or the GUI's decode would reject it) -- see step 4. ===")
        return 1

    modem_sleep_signature = result.successes < result.attempts and (
        result.rtt_max_ms is not None and result.rtt_max_ms >= _RTT_MODEM_SLEEP_MS
    )
    if modem_sleep_signature:
        print(
            "\n=== VERDICT: modem-sleep signature -- link works, but not reliably enough for the GUI. ===\n"
            f"{result.successes}/{result.attempts} pings got a reply, and the replies that arrived took up to "
            f"{result.rtt_max_ms:.0f}ms. This is not a wiring, address, or firmware-logic problem: it's the "
            "ESP32's WiFi radio powering down between packets and taking too long to wake, so most packets "
            "are lost and the ones that land arrive too late for the GUI's link timeout. Fix in firmware: "
            "disable WiFi power save (esp_wifi_set_ps(WIFI_PS_NONE), called once WiFi association succeeds) "
            "-- see docs/HOW_TO_USE.md's link troubleshooting section for the full explanation and expected "
            "RTT after the fix."
        )
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
