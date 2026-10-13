import copy
from pathlib import Path
import math
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from roboarm_nodes.configuration import load, validate, save, check_targets, cylindrical_pose

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def config():
    return load(ROOT / 'src/roboarm_bringup/config/hardware.yaml')


def test_example_is_simulation_only(config):
    validate(config)
    with pytest.raises(ValueError):
        validate(config, physical=True)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), True, '0.1'])
def test_bad_target_rejected(config, value):
    with pytest.raises(ValueError):
        check_targets(config, value, 0)


def test_limits_and_reference(config):
    check_targets(config, 0, -1.57)
    check_targets(config, 0.3, 1.57)
    with pytest.raises(ValueError):
        check_targets(config, 0.301, 0)
    with pytest.raises(ValueError):
        check_targets(config, 0.1, 1.58)
    config['joints']['z_joint']['initial_position'] = 0.31
    with pytest.raises(ValueError):
        validate(config)


def test_no_switches_required_after_gpio_assignment(config):
    config['commissioned'] = True
    for name, pins in [('z_joint', [20,21,17,27,22,23]), ('rotation_joint', [5,6,12,13,16,19])]:
        for key, pin in zip(('direction_pin','step_pin','ms1_pin','ms2_pin','ms3_pin','enable_pin'), pins):
            config['joints'][name][key] = pin
    validate(config, physical=True)


def test_duplicate_gpio_rejected(config):
    config['joints']['rotation_joint']['step_pin'] = 21
    with pytest.raises(ValueError, match='more than once'):
        validate(config)


def test_speed_cannot_exceed_driver(config):
    config['joints']['z_joint']['max_velocity'] = 1
    with pytest.raises(ValueError, match='pulse-rate'):
        validate(config)


def test_save_does_not_persist_reference_or_modify_active_snapshot(config, tmp_path):
    active = copy.deepcopy(config)
    config['reference_confirmed'] = True
    config['joints']['z_joint']['upper'] = 0.25
    path = tmp_path / 'hardware.yaml'
    save(path, config)
    assert load(path)['reference_confirmed'] is False
    assert active['joints']['z_joint']['upper'] == 0.3
    assert load(path)['joints']['z_joint']['upper'] == 0.25


def test_circular_reach_and_yaw():
    x,y,z,qz,qw = cylindrical_pose(0.3,0.1,0.05,math.pi/2)
    assert x == pytest.approx(0, abs=1e-12)
    assert y == pytest.approx(0.3)
    assert z == pytest.approx(0.15)
    assert qz*qz + qw*qw == pytest.approx(1)
