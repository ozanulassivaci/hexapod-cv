"""UDPRobotLink: sends commands to the robot over UDP and receives telemetry
on a bound listen socket. Non-blocking send, a background thread resends
the last-sent command at a fixed interval (the heartbeat -- see
docs/protocol.md Section 2), and a second background thread decodes
inbound telemetry.

Threads start immediately on construction (no separate start() call) so the
object is live and already heartbeating -- with StopCommand, the safest
possible default -- from the moment it exists.
"""

import logging
import socket
import threading

from transport.generated_constants import HEARTBEAT_INTERVAL_S, LINK_TIMEOUT_S
from transport.link import RobotLink
from transport.protocol import Command, ProtocolError, StopCommand, decode_telemetry, encode_command, next_sequence

logger = logging.getLogger(__name__)

_RECV_BUFFER_SIZE = 65535
_SOCKET_POLL_TIMEOUT_S = 0.2


class UDPRobotLink(RobotLink):
    def __init__(
        self,
        robot_host: str,
        robot_port: int,
        listen_port: int,
        heartbeat_interval_s: float = HEARTBEAT_INTERVAL_S,
        connection_timeout_s: float = LINK_TIMEOUT_S,
    ) -> None:
        super().__init__(connection_timeout_s=connection_timeout_s)
        self._robot_addr = (robot_host, robot_port)
        self._heartbeat_interval_s = heartbeat_interval_s

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", listen_port))
        self._socket.settimeout(_SOCKET_POLL_TIMEOUT_S)

        self._send_lock = threading.Lock()
        self._seq = 0
        self._last_command: Command = StopCommand()

        self._stop_event = threading.Event()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._heartbeat_thread.start()
        self._receive_thread.start()

    def send(self, command: Command) -> int:
        with self._send_lock:
            self._last_command = command
            return self._transmit_locked(command)

    def close(self) -> None:
        self._stop_event.set()
        self._heartbeat_thread.join(timeout=2)
        self._receive_thread.join(timeout=2)
        self._socket.close()

    def _transmit_locked(self, command: Command) -> int:
        """Caller must hold self._send_lock. Assigns and returns the next
        sequence number and sends the packet; a send failure is logged, not
        raised -- the next heartbeat tick will simply try again."""
        seq = self._seq
        self._seq = next_sequence(self._seq)
        data = encode_command(command, seq)
        try:
            self._socket.sendto(data, self._robot_addr)
        except OSError as exc:
            logger.warning("UDP send to %s failed: %s", self._robot_addr, exc)
        return seq

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._send_lock:
                self._transmit_locked(self._last_command)
            self._stop_event.wait(self._heartbeat_interval_s)

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _addr = self._socket.recvfrom(_RECV_BUFFER_SIZE)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    break
                continue

            try:
                telemetry = decode_telemetry(data)
            except ProtocolError as exc:
                logger.warning("dropped malformed telemetry packet: %s", exc)
                continue

            self._record_telemetry(telemetry)
