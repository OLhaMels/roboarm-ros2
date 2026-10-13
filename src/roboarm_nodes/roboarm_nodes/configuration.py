"""Shared validation for launch, Web UI and task manager; ROS-independent."""
import copy
import math
import os
from pathlib import Path
import tempfile
import yaml

JOINTS = ('z_joint', 'rotation_joint')
PINS = ('direction_pin', 'step_pin', 'ms1_pin', 'ms2_pin', 'ms3_pin',
        'enable_pin', 'min_limit_pin', 'max_limit_pin')


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


def validate(data, physical=False):
    if not isinstance(data, dict) or set(data.get('joints', {})) != set(JOINTS):
        raise ValueError('Exactly z_joint and rotation_joint are required')
    for name in ('commissioned', 'reference_confirmed'):
        if not isinstance(data.get(name), bool):
            raise ValueError(f'{name} must be true or false')
    if not isinstance(data.get('gpio_chip'), str) or not data['gpio_chip'].startswith('/dev/gpiochip'):
        raise ValueError('gpio_chip must be an absolute /dev/gpiochip path')
    if not 0.05 <= finite(data.get('watchdog_seconds'), 'watchdog_seconds') <= 1.0:
        raise ValueError('watchdog_seconds must be 0.05..1')
    if not 0.1 <= finite(data.get('arm_radius'), 'arm_radius') <= 2.0:
        raise ValueError('arm_radius must be 0.1..2 metres')
    if not 0 <= finite(data.get('base_height'), 'base_height') <= 2.0:
        raise ValueError('base_height must be 0..2 metres')
    used = set()
    for name in JOINTS:
        j = data['joints'][name]
        for key in ('lower', 'upper', 'initial_position', 'max_velocity',
                    'max_acceleration', 'steps_per_unit', 'max_step_rate'):
            finite(j.get(key), f'{name}.{key}')
        if not j['lower'] < j['upper'] or not j['lower'] <= j['initial_position'] <= j['upper']:
            raise ValueError(f'{name}: initial_position must be within ordered limits')
        bound = 2.0 if name == 'z_joint' else 2 * math.pi
        if j['lower'] < -bound or j['upper'] > bound:
            raise ValueError(f'{name}: limits outside supported range +/-{bound}')
        if j['steps_per_unit'] <= 0 or not 1 <= j['max_step_rate'] <= 500:
            raise ValueError(f'{name}: positive scale and step rate 1..500 required')
        if not 0 < j['max_velocity'] <= 0.8 * j['max_step_rate'] / j['steps_per_unit']:
            raise ValueError(f'{name}: velocity exceeds 80% of pulse-rate capacity')
        if not 0 < j['max_acceleration'] <= 600 / j['steps_per_unit']:
            raise ValueError(f'{name}: acceleration exceeds driver ramp capacity')
        if not isinstance(j.get('direction_inverted'), bool):
            raise ValueError(f'{name}: direction_inverted must be boolean')
        for key in PINS:
            pin = j.get(key)
            if isinstance(pin, bool) or not isinstance(pin, int) or not -1 <= pin <= 27:
                raise ValueError(f'{name}.{key}: use integer BCM offset -1..27')
            if physical and pin == -1 and key not in ('min_limit_pin', 'max_limit_pin'):
                raise ValueError(f'{name}.{key}: assign a GPIO before enabling hardware')
            if pin >= 0:
                if pin in used:
                    raise ValueError(f'GPIO {pin} is assigned more than once')
                used.add(pin)
    if physical and not data['commissioned']:
        raise ValueError('Set commissioned only after calibrating geometry, wiring and direction')
    return data


def load(path, physical=False):
    return validate(yaml.safe_load(Path(path).read_text(encoding='utf-8-sig')), physical)


def save(path, data):
    data = copy.deepcopy(validate(data))
    data['reference_confirmed'] = False  # Never persist a startup reference acknowledgement.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.roboarm-', suffix='.yaml')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def check_targets(config, z, rotation):
    for name, value in zip(JOINTS, (z, rotation)):
        value = finite(value, name)
        j = config['joints'][name]
        if not j['lower'] <= value <= j['upper']:
            raise ValueError(f'{name} outside active travel limits')


def cylindrical_pose(radius, height, z, rotation):
    return (radius * math.cos(rotation), radius * math.sin(rotation), height + z,
            math.sin(rotation / 2), math.cos(rotation / 2))
