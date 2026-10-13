"""Offline checks: pytest + xacro + PyYAML, no ROS installation required."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/roboarm_nodes'))
from roboarm_nodes.configuration import load, save


@pytest.mark.parametrize('mode', ['mock', 'gpio'])
def test_model_interfaces_and_controller_joints(mode):
    config = ROOT / 'src/roboarm_bringup/config/hardware.yaml'
    document = xacro.process_file(str(ROOT / 'src/roboarm_bringup/urdf/roboarm.urdf.xacro'),
        mappings={'hardware_mode': mode, 'hardware_config': config.as_posix()})
    robot = ET.fromstring(document.toxml())
    active = {j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed'}
    assert active == {'z_joint', 'rotation_joint', 'gripper_joint'}
    systems = {s.get('name'): s for s in robot.findall('ros2_control')}
    plugin = systems['ArmSystem'].findtext('hardware/plugin')
    assert plugin == ('roboarm_hardware/GpioSystem' if mode == 'gpio' else 'mock_components/GenericSystem')
    assert systems['ExampleGripper'].findtext('hardware/plugin') == 'mock_components/GenericSystem'
    assert systems['ArmSystem'].find("hardware/param[@name='reference_confirmed']").text == 'false'
    controllers = yaml.safe_load((ROOT / 'src/roboarm_bringup/config/controllers.yaml').read_text())
    assert controllers['arm_controller']['ros__parameters']['joints'] == ['z_joint', 'rotation_joint']
    links = {link.get('name') for link in robot.findall('link')}
    for joint in robot.findall('joint'):
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links


def test_saved_limits_reach_urdf_and_hardware(tmp_path):
    data = load(ROOT / 'src/roboarm_bringup/config/hardware.yaml')
    data['joints']['z_joint']['upper'] = 0.123
    data['joints']['rotation_joint']['lower'] = -0.4
    path = tmp_path / 'changed.yaml'
    save(path, data)
    document = xacro.process_file(str(ROOT / 'src/roboarm_bringup/urdf/roboarm.urdf.xacro'),
        mappings={'hardware_mode': 'mock', 'hardware_config': path.as_posix()})
    robot = ET.fromstring(document.toxml())
    assert float(robot.find("joint[@name='z_joint']/limit").get('upper')) == 0.123
    assert float(robot.find("ros2_control/joint[@name='z_joint']/command_interface/param[@name='max']").text) == 0.123
    assert float(robot.find("joint[@name='rotation_joint']/limit").get('lower')) == -0.4
