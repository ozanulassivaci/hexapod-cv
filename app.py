"""Operator GUI entry point.

    python app.py
    python app.py --link-mode sim
    python app.py --camera-mode synthetic   # closes vision -> tracker -> gait end to end, no hardware

Wires a camera stream, HSVDetector, a RobotLink, and the Tracker together
and launches MainWindow. Defaults to MockRobotLink (operator_config.yaml's
link.mode) -- no hardware and no camera required to run; both degrade to
a visible "disconnected"/"waiting for camera" state rather than an error.
--link-mode overrides link.mode from the config file for the session,
without editing the YAML -- selectable from config or CLI, per the design
discussion this was built from.

--camera-mode synthetic swaps MJPEGStream for SyntheticCameraStream (see
simulator/synthetic_camera.py), which renders a target whose apparent
position reacts to SimRobotLink's actual heading -- the same real
HSVDetector and Tracker code run against it as against a real camera, so
switching AUTO_TRACK on with this flag closes the whole vision -> tracker
-> gait loop with nothing real anywhere in the chain. Implies --link-mode
sim (a synthetic camera reacting to a non-simulated robot's heading makes
no sense) unless sim was already selected.
"""

import argparse
import sys

from PySide6.QtWidgets import QApplication

from control.tracker import Tracker
from operator_config import OperatorConfig, load_operator_config
from perception.hsv_detector import HSVDetector
from simulator.sim_link import SimRobotLink
from simulator.synthetic_camera import SyntheticCameraStream, SyntheticTarget
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


def build_stream(config: OperatorConfig, link: RobotLink, camera_mode: str, target_bearing_deg: float):
    if camera_mode == "synthetic":
        return SyntheticCameraStream(link, [SyntheticTarget(world_bearing_deg=target_bearing_deg)])
    return MJPEGStream(
        config.stream.url,
        retry_backoff_base_s=config.stream.retry_backoff_base_s,
        retry_backoff_max_s=config.stream.retry_backoff_max_s,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="hexapod-cv operator GUI")
    parser.add_argument(
        "--link-mode",
        choices=("mock", "sim", "udp"),
        default=None,
        help="override operator_config.yaml's link.mode for this run",
    )
    parser.add_argument(
        "--camera-mode",
        choices=("real", "synthetic"),
        default="real",
        help="'synthetic' renders a target that reacts to the simulator's live heading -- "
        "no camera or hardware anywhere in the loop. Implies --link-mode sim.",
    )
    parser.add_argument(
        "--target-bearing",
        type=float,
        default=25.0,
        help="world bearing (degrees) of the synthetic target, only used with --camera-mode synthetic",
    )
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args(sys.argv[1:])
    config = load_operator_config()
    if args.link_mode is not None:
        config.link.mode = args.link_mode
    if args.camera_mode == "synthetic" and config.link.mode != "sim":
        print("--camera-mode synthetic requires the simulator link -- overriding link.mode to 'sim'")
        config.link.mode = "sim"

    detector = HSVDetector(
        lower=config.detection.hsv.lower,
        upper=config.detection.hsv.upper,
        min_contour_area=config.detection.min_contour_area,
        label=config.detection.label,
    )
    link = build_link(config)
    stream = build_stream(config, link, args.camera_mode, args.target_bearing).start()
    tracker = Tracker(config.tracker)

    app = QApplication(sys.argv)
    window = MainWindow(config, stream, detector, link, tracker)
    window.resize(1100, 720)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
