"""Configuration for both ends.

One JSON file is shared by the transmitter and the receiver, overlaid on the
built-in defaults below, so only the keys you want to change need to be
present. Unknown keys are an error: a typo cannot silently disable a
fail-safe. Links in the file are merged with the defaults by name, so
``{"links": [{"name": "wifi", "robot_host": "10.0.0.5"}]}`` changes only the
robot's address; give a link ``"enabled": false`` to switch it off.
"""
from __future__ import annotations

import dataclasses
import json
import os
from dataclasses import dataclass, fields, is_dataclass, replace
from typing import Any, Dict, List, Optional, Tuple

from .joystick import JoystickConfig
from .links import LinkSpec
from .obstacle import ObstacleConfig
from .servo import DriveCalibration
from .telemetry import TelemetryConfig


class ConfigError(ValueError):
    """The configuration is invalid."""


@dataclass(frozen=True)
class ReceiverConfig:
    control_rate_hz: float = 50.0
    fresh_for_s: float = 0.5        # a link is trusted while its newest frame is younger than this
    status_interval_s: float = 1.0  # how often the robot reports back to the transmitter

    def __post_init__(self) -> None:
        if min(self.control_rate_hz, self.fresh_for_s, self.status_interval_s) <= 0:
            raise ValueError("receiver rates and intervals must be positive")


@dataclass(frozen=True)
class TransmitterConfig:
    rate_hz: float = 50.0
    status_print_interval_s: float = 2.0
    shutdown_frames: int = 5        # e-stop frames sent on every link when the transmitter exits

    def __post_init__(self) -> None:
        if self.rate_hz <= 0 or self.status_print_interval_s <= 0:
            raise ValueError("transmitter rate and interval must be positive")
        if self.shutdown_frames < 0:
            raise ValueError("shutdown_frames must not be negative")


DEFAULT_LINKS: Tuple[LinkSpec, ...] = (
    LinkSpec(type="udp", name="wifi", priority=1, robot_host="192.168.0.201", port=5555,
             max_send_hz=50.0),
    LinkSpec(type="serial", name="radio", priority=3, tx_port="/dev/serial0", rx_port="/dev/ttyTHS1",
             baudrate=9600, max_send_hz=15.0),
    LinkSpec(type="modbus", name="modbus", priority=2, enabled=False, robot_host="192.168.0.201",
             port=5020, base_address=128, max_send_hz=20.0),
)


@dataclass(frozen=True)
class Config:
    links: Tuple[LinkSpec, ...] = DEFAULT_LINKS
    drive: DriveCalibration = DriveCalibration()
    joystick: JoystickConfig = JoystickConfig()
    receiver: ReceiverConfig = ReceiverConfig()
    transmitter: TransmitterConfig = TransmitterConfig()
    telemetry: TelemetryConfig = TelemetryConfig()
    obstacle: ObstacleConfig = ObstacleConfig()

    def enabled_links(self) -> List[LinkSpec]:
        return [spec for spec in self.links if spec.enabled]

    def validate(self) -> None:
        names = [spec.name for spec in self.links]
        if len(set(names)) != len(names):
            raise ConfigError("link names must be unique")
        enabled = self.enabled_links()
        if not enabled:
            raise ConfigError("no links are enabled")
        priorities = [spec.priority for spec in enabled]
        if len(set(priorities)) != len(priorities):
            raise ConfigError("enabled links must have distinct priorities")
        for spec in self.links:
            if spec.type not in ("udp", "serial", "modbus"):
                raise ConfigError(f"link {spec.name!r}: unknown type {spec.type!r}")

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


def merge(instance: Any, data: Any) -> Any:
    """Return a copy of a config dataclass with ``data`` overlaid, recursively."""
    if not isinstance(data, dict):
        raise ConfigError(f"{type(instance).__name__}: expected an object, got {type(data).__name__}")
    known = {f.name for f in fields(instance)}
    changes: Dict[str, Any] = {}
    for key, value in data.items():
        if key not in known:
            raise ConfigError(f"unknown setting {key!r} in {type(instance).__name__}; "
                              f"known settings: {', '.join(sorted(known))}")
        current = getattr(instance, key)
        if key == "links":
            changes[key] = _merge_links(current, value)
        elif is_dataclass(current) and not isinstance(current, type):
            changes[key] = merge(current, value)
        elif isinstance(current, tuple):
            if not isinstance(value, (list, tuple)):
                raise ConfigError(f"{key!r} must be a list")
            changes[key] = tuple(value)
        else:
            changes[key] = value
    try:
        return replace(instance, **changes)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{type(instance).__name__}: {exc}") from exc


def _merge_links(current: Tuple[LinkSpec, ...], items: Any) -> Tuple[LinkSpec, ...]:
    if not isinstance(items, (list, tuple)):
        raise ConfigError("'links' must be a list of link objects")
    known = {f.name for f in fields(LinkSpec)}
    by_name = {spec.name: spec for spec in current}
    order = [spec.name for spec in current]
    for item in items:
        if not isinstance(item, dict) or "name" not in item:
            raise ConfigError("each link must be an object with a 'name'")
        unknown = sorted(set(item) - known)
        if unknown:
            raise ConfigError(f"link {item['name']!r}: unknown setting(s) {unknown}; "
                              f"known settings: {sorted(known)}")
        try:
            if item["name"] in by_name:
                by_name[item["name"]] = replace(by_name[item["name"]], **item)
            else:
                by_name[item["name"]] = LinkSpec(**item)
                order.append(item["name"])
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"link {item['name']!r}: {exc}") from exc
    return tuple(by_name[name] for name in order)


def load_config(path: Optional[str] = None) -> Config:
    """Load ``path`` over the defaults; with no path, return the defaults."""
    cfg = Config()
    if path:
        full = os.path.expanduser(path)
        try:
            with open(full) as handle:
                data = json.load(handle)
        except OSError as exc:
            raise ConfigError(f"cannot read {full}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise ConfigError(f"{full}: invalid JSON: {exc}") from exc
        cfg = merge(cfg, data)
    cfg.validate()
    return cfg
