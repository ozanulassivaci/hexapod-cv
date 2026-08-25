from dataclasses import dataclass, field


@dataclass
class StreamConfig:
    url: str = "http://192.168.0.52:4747/video/640x480"
    retry_backoff_base_s: float = 1.0
    retry_backoff_max_s: float = 30.0


@dataclass
class HSVRange:
    lower: tuple[int, int, int] = (100, 150, 50)
    upper: tuple[int, int, int] = (140, 255, 255)


@dataclass
class DetectionConfig:
    hsv: HSVRange = field(default_factory=HSVRange)
    min_contour_area: int = 500
    label: str = "blue_object"


@dataclass
class WindowConfig:
    width: int = 640
    height: int = 480
    title: str = "hexapod-cv"


@dataclass
class AppConfig:
    stream: StreamConfig = field(default_factory=StreamConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    window: WindowConfig = field(default_factory=WindowConfig)


def load_config() -> AppConfig:
    return AppConfig()
