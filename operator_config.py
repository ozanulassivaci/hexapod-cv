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
    mode: str = "mock"  # "mock" | "sim" | "udp"
    udp: UDPLinkConfig = field(default_factory=UDPLinkConfig)

    def __post_init__(self) -> None:
        if self.mode not in ("mock", "sim", "udp"):
            raise ValueError(f"link.mode must be 'mock', 'sim', or 'udp', got {self.mode!r}")


@dataclass
class UIConfig:
    tick_interval_s: float = 0.033
    window_width: int = 640
    window_height: int = 480


@dataclass
class BenchConfig:
    """Tunables for the Bench Test tab. neutral_pulse_us is deliberately
    not here -- it's transport.protocol.NEUTRAL_PULSE_US, a wire-level
    convention, not an operator preference. dwell_timeout_s must stay well
    under BENCH_ARM_TIMEOUT_S (transport/constants.yaml) or bench mode can
    auto-disarm before the dwell guard ever fires -- BenchTab warns at
    construction if that's violated."""

    dwell_timeout_s: float = 8.0
    nudge_step_us: int = 10
    default_sweep_min_us: int = 1400
    default_sweep_max_us: int = 1600
    default_sweep_duration_s: float = 4.0


@dataclass
class OperatorConfig:
    stream: StreamConfig = field(default_factory=StreamConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    link: LinkConfig = field(default_factory=LinkConfig)
    tracker: TrackerConfig = field(
        default_factory=lambda: TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0)
    )
    ui: UIConfig = field(default_factory=UIConfig)
    bench: BenchConfig = field(default_factory=BenchConfig)


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
        lost_target_hold_s=data.get("lost_target_hold_s", defaults.lost_target_hold_s),
    )


def _ui_from_dict(data: dict) -> UIConfig:
    defaults = UIConfig()
    return UIConfig(
        tick_interval_s=data.get("tick_interval_s", defaults.tick_interval_s),
        window_width=data.get("window_width", defaults.window_width),
        window_height=data.get("window_height", defaults.window_height),
    )


def _bench_from_dict(data: dict) -> BenchConfig:
    defaults = BenchConfig()
    return BenchConfig(
        dwell_timeout_s=data.get("dwell_timeout_s", defaults.dwell_timeout_s),
        nudge_step_us=data.get("nudge_step_us", defaults.nudge_step_us),
        default_sweep_min_us=data.get("default_sweep_min_us", defaults.default_sweep_min_us),
        default_sweep_max_us=data.get("default_sweep_max_us", defaults.default_sweep_max_us),
        default_sweep_duration_s=data.get(
            "default_sweep_duration_s", defaults.default_sweep_duration_s
        ),
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
        bench=_bench_from_dict(raw.get("bench", {})),
    )
