"""Config for the operator GUI (app.py). Named/separate from config.py,
which stays scoped to the Phase 1 camera-only viewer (main.py, tune_hsv.py)
and is not touched here.

Loaded from operator_config.yaml. Every field has a Python-side default
matching that file, so a missing section (or a missing operator_config.yaml
entirely) degrades to sane defaults rather than crashing -- the no-hardware
path is the primary one this app runs against, and that shouldn't require
a config file to exist.
"""

from dataclasses import dataclass, field

import yaml

from config import DetectionConfig, HSVRange, StreamConfig
from control.tracker import TrackerConfig

DEFAULT_CONFIG_PATH = "operator_config.yaml"


@dataclass
class UDPLinkConfig:
    robot_host: str = "192.168.0.50"
    robot_port: int = 9000
    listen_port: int = 9001


@dataclass
class LinkConfig:
    mode: str = "mock"  # "mock" | "udp"
    udp: UDPLinkConfig = field(default_factory=UDPLinkConfig)

    def __post_init__(self) -> None:
        if self.mode not in ("mock", "udp"):
            raise ValueError(f"link.mode must be 'mock' or 'udp', got {self.mode!r}")


@dataclass
class UIConfig:
    tick_interval_s: float = 0.033
    window_width: int = 640
    window_height: int = 480


@dataclass
class OperatorConfig:
    stream: StreamConfig = field(default_factory=StreamConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    tracker: TrackerConfig = field(
        default_factory=lambda: TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0)
    )
    ui: UIConfig = field(default_factory=UIConfig)


def _stream_from_dict(data: dict) -> StreamConfig:
    defaults = StreamConfig()
    return StreamConfig(
        url=data.get("url", defaults.url),
        retry_backoff_base_s=data.get("retry_backoff_base_s", defaults.retry_backoff_base_s),
        retry_backoff_max_s=data.get("retry_backoff_max_s", defaults.retry_backoff_max_s),
    )


def _detection_from_dict(data: dict) -> DetectionConfig:
    defaults = DetectionConfig()
    hsv_data = data.get("hsv", {})
    hsv = HSVRange(
        lower=tuple(hsv_data.get("lower", defaults.hsv.lower)),
        upper=tuple(hsv_data.get("upper", defaults.hsv.upper)),
    )
    return DetectionConfig(
        hsv=hsv,
        min_contour_area=data.get("min_contour_area", defaults.min_contour_area),
        label=data.get("label", defaults.label),
    )


def _link_from_dict(data: dict) -> LinkConfig:
    udp_data = data.get("udp", {})
    udp_defaults = UDPLinkConfig()
    udp = UDPLinkConfig(
        robot_host=udp_data.get("robot_host", udp_defaults.robot_host),
        robot_port=udp_data.get("robot_port", udp_defaults.robot_port),
        listen_port=udp_data.get("listen_port", udp_defaults.listen_port),
    )
    return LinkConfig(mode=data.get("mode", "mock"), udp=udp)


def _tracker_from_dict(data: dict) -> TrackerConfig:
    defaults = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0)
    return TrackerConfig(
        dead_zone=data.get("dead_zone", defaults.dead_zone),
        turn_gain=data.get("turn_gain", defaults.turn_gain),
        turn_speed=data.get("turn_speed", defaults.turn_speed),
    )


def _ui_from_dict(data: dict) -> UIConfig:
    defaults = UIConfig()
    return UIConfig(
        tick_interval_s=data.get("tick_interval_s", defaults.tick_interval_s),
        window_width=data.get("window_width", defaults.window_width),
        window_height=data.get("window_height", defaults.window_height),
    )


def load_operator_config(path: str = DEFAULT_CONFIG_PATH) -> OperatorConfig:
    try:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return OperatorConfig()

    return OperatorConfig(
        stream=_stream_from_dict(raw.get("stream", {})),
        detection=_detection_from_dict(raw.get("detection", {})),
        link=_link_from_dict(raw.get("link", {})),
        tracker=_tracker_from_dict(raw.get("tracker", {})),
        ui=_ui_from_dict(raw.get("ui", {})),
    )
