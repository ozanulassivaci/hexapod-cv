"""Operator GUI entry point.

    python app.py
    python app.py --link-mode sim

Wires MJPEGStream, HSVDetector, a RobotLink, and the Tracker together and
launches MainWindow. Defaults to MockRobotLink (operator_config.yaml's
link.mode) -- no hardware and no camera required to run; both degrade to
a visible "disconnected"/"waiting for camera" state rather than an error.
--link-mode overrides link.mode from the config file for the session,
without editing the YAML -- selectable from config or CLI, per the design
discussion this was built from.
"""

import argparse
import sys

from PySide6.QtWidgets import QApplication

from control.tracker import Tracker
from operator_config import OperatorConfig, load_operator_config
from perception.hsv_detector import HSVDetector
from simulator.sim_link import SimRobotLink
from stream.mjpeg_stream import MJPEGStream
from transport.link import RobotLink
from transport.mock_link import MockRobotLink
from transport.udp_link import UDPRobotLink
from ui.main_window import MainWindow


def build_link(config: OperatorConfig) -> RobotLink:
    if config.link.mode == "udp":
        return UDPRobotLink(
            config.link.udp.robot_host,
            config.link.udp.robot_port,
            config.link.udp.listen_port,
        )
    if config.link.mode == "sim":
        return SimRobotLink()
    return MockRobotLink()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="hexapod-cv operator GUI")
    parser.add_argument(
        "--link-mode",
        choices=("mock", "sim", "udp"),
        default=None,
        help="override operator_config.yaml's link.mode for this run",
    )
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args(sys.argv[1:])
    config = load_operator_config()
    if args.link_mode is not None:
        config.link.mode = args.link_mode

    detector = HSVDetector(
        lower=config.detection.hsv.lower,
        upper=config.detection.hsv.upper,
        min_contour_area=config.detection.min_contour_area,
        label=config.detection.label,
    )
    stream = MJPEGStream(
        config.stream.url,
        retry_backoff_base_s=config.stream.retry_backoff_base_s,
        retry_backoff_max_s=config.stream.retry_backoff_max_s,
    ).start()
    link = build_link(config)
    tracker = Tracker(config.tracker)

    app = QApplication(sys.argv)
    window = MainWindow(config, stream, detector, link, tracker)
    window.resize(1100, 720)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
