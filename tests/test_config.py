import json

import pytest

from jetnano_control.config import Config, ConfigError, load_config, merge


def write(tmp_path, data):
    path = tmp_path / "config.json"
    path.write_text(json.dumps(data))
    return str(path)


def test_defaults_validate():
    cfg = load_config(None)
    assert [spec.name for spec in cfg.enabled_links()] == ["wifi", "radio"]
    assert cfg.drive.throttle.channel == 0 and cfg.receiver.fresh_for_s == 0.5


def test_links_merge_by_name_and_nested_overrides(tmp_path):
    path = write(tmp_path, {
        "links": [
            {"name": "wifi", "robot_host": "10.0.0.5"},
            {"name": "modbus", "enabled": True},
            {"name": "lan2", "type": "udp", "priority": 5, "robot_host": "10.0.0.6", "port": 5556},
        ],
        "drive": {"throttle": {"center": 88}, "throttle_limit": 0.4},
        "receiver": {"fresh_for_s": 0.8},
        "joystick": {"axes": [3, 2, 1, 0]},
    })
    cfg = load_config(path)
    by_name = {spec.name: spec for spec in cfg.links}
    assert by_name["wifi"].robot_host == "10.0.0.5" and by_name["wifi"].port == 5555
    assert by_name["modbus"].enabled and by_name["lan2"].priority == 5
    assert [spec.name for spec in cfg.links] == ["wifi", "radio", "modbus", "lan2"]
    assert cfg.drive.throttle.center == 88 and cfg.drive.throttle.minimum == 65
    assert cfg.drive.throttle_limit == 0.4 and cfg.receiver.fresh_for_s == 0.8
    assert cfg.joystick.axes == (3, 2, 1, 0)


@pytest.mark.parametrize("data, message", [
    ({"reciever": {}}, "unknown setting 'reciever'"),
    ({"links": [{"name": "wifi", "robot_hots": "x"}]}, "unknown setting"),
    ({"links": [{"type": "udp"}]}, "'name'"),
    ({"links": [{"name": "new", "priority": 9}]}, "type"),
    ({"drive": {"throttle_limit": 0}}, "throttle_limit"),
    ({"drive": {"throttle": {"minimum": 200}}}, "channel 0"),
    ({"links": [{"name": "radio", "priority": 1}]}, "distinct priorities"),
    ({"links": [{"name": "wifi", "enabled": False}, {"name": "radio", "enabled": False}]}, "no links are enabled"),
    ({"links": [{"name": "wifi", "type": "carrier-pigeon"}]}, "unknown type"),
    ({"joystick": {"axes": [0, 1]}}, "four entries"),
    ({"receiver": "fast"}, "expected an object"),
])
def test_bad_configs_are_rejected(tmp_path, data, message):
    with pytest.raises(ConfigError, match=message):
        load_config(write(tmp_path, data))


def test_unreadable_or_invalid_json(tmp_path):
    with pytest.raises(ConfigError, match="cannot read"):
        load_config(str(tmp_path / "missing.json"))
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    with pytest.raises(ConfigError, match="invalid JSON"):
        load_config(str(path))


def test_to_dict_round_trips_through_merge():
    cfg = Config()
    assert merge(Config(), cfg.to_dict()) == cfg
