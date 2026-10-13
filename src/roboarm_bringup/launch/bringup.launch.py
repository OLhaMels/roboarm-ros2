from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from roboarm_nodes.configuration import load


def setup(context):
    share = Path(get_package_share_directory('roboarm_bringup'))
    mode = LaunchConfiguration('hardware_mode').perform(context)
    if mode not in ('mock', 'gpio'):
        raise ValueError('hardware_mode must be mock or gpio')
    demo = LaunchConfiguration('demo_perception').perform(context).lower() == 'true'
    if mode == 'gpio' and demo:
        raise ValueError('GPIO motion must not use synthetic perception; set demo_perception:=false')
    hw_path = LaunchConfiguration('hardware_config').perform(context)
    if not hw_path:
        target = Path.home() / '.config/roboarm/hardware.yaml'
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copyfile(share / 'config/hardware.yaml', target)
        hw_path = str(target)
    hw = load(hw_path, physical=mode == 'gpio')
    reference = LaunchConfiguration('reference_confirmed').perform(context).lower()
    if mode == 'gpio' and reference != 'true':
        raise ValueError('Manually set both axes to configured initial_position, then pass reference_confirmed:=true')
    description = {'robot_description': ParameterValue(xacro.process_file(
        str(share / 'urdf/roboarm.urdf.xacro'),
        mappings={'hardware_mode': mode, 'hardware_config': hw_path, 'reference_confirmed': reference}).toxml(), value_type=str)}
    moveit = yaml.safe_load((share / 'config/moveit.yaml').read_text())
    for name, joint in hw['joints'].items():
        moveit['robot_description_planning']['joint_limits'][name] = {
            'has_velocity_limits': True, 'max_velocity': float(joint['max_velocity']),
            'has_acceleration_limits': True, 'max_acceleration': float(joint['max_acceleration']),
        }
    srdf = ET.parse(share / 'config/robot.srdf').getroot()
    for joint in srdf.findall('group_state/joint'):
        joint.set('value', str(hw['joints'][joint.get('name')]['initial_position']))
    semantic = {'robot_description_semantic': ParameterValue(ET.tostring(srdf, encoding='unicode'), value_type=str)}
    shared = {'hardware_config': hw_path, 'hardware_mode': mode}
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[str(share / 'config/controllers.yaml')], output='screen',
                   remappings=[('robot_description', '/robot_description')])
    # Start higher layers only after all controller spawners have succeeded.
    spawners = [Node(package='controller_manager', executable='spawner',
                     arguments=[name, '--controller-manager', '/controller_manager',
                                '--controller-manager-timeout', '60'], output='screen')
                for name in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')]
    application = [
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[description, semantic, moveit], output='screen'),
        Node(package='roboarm_nodes', executable='task_manager_node', parameters=[shared], output='screen'),
        Node(package='roboarm_nodes', executable='camera_node', output='screen',
             parameters=[{'demo': demo}, LaunchConfiguration('perception_config')]),
        Node(package='roboarm_nodes', executable='vision_node', output='screen',
             parameters=[{'demo': demo, 'demo_radius': float(hw['arm_radius']),
                          'demo_height': float(hw['base_height'])}, LaunchConfiguration('perception_config')]),
        Node(package='roboarm_nodes', executable='web_ui', output='screen',
             parameters=[shared, {'host': LaunchConfiguration('web_host')}]),
    ]
    if LaunchConfiguration('video_stream').perform(context).lower() == 'true':
        # The old upd-rasp-stream-tcp view: camera + QR overlay as H.264 over TCP.
        application.append(Node(package='roboarm_nodes', executable='stream_node', output='screen',
                                parameters=[LaunchConfiguration('perception_config')]))

    def after(next_actions):
        def callback(event, _context):
            return next_actions if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Controller startup failed'))]
        return callback

    handlers = [RegisterEventHandler(OnProcessExit(target_action=spawners[i],
                on_exit=after([spawners[i + 1]] if i < 2 else application))) for i in range(3)]
    return [*handlers, Node(package='robot_state_publisher', executable='robot_state_publisher',
                           parameters=[description]), manager, spawners[0]]


def generate_launch_description():
    share = Path(get_package_share_directory('roboarm_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware_mode', default_value='mock'),
        DeclareLaunchArgument('hardware_config', default_value=''),
        DeclareLaunchArgument('reference_confirmed', default_value='false'),
        DeclareLaunchArgument('perception_config', default_value=str(share / 'config/perception.yaml')),
        DeclareLaunchArgument('demo_perception', default_value='true'),
        DeclareLaunchArgument('web_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('video_stream', default_value='false'),
        OpaqueFunction(function=setup),
    ])
