import copy
import json
import math
import threading
import time
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from roboarm_interfaces.action import PickPlace, MoveJoints
from .configuration import load, check_targets, JOINTS


class Cancelled(Exception):
    pass


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.declare_parameter('hardware_config', '')
        self.declare_parameter('hardware_mode', 'mock')
        self.config = load(self.get_parameter('hardware_config').value)
        self.mode = self.get_parameter('hardware_mode').value
        self.group = ReentrantCallbackGroup()
        self.motion_lock = threading.Lock()
        self.fault = ''
        self.stage = 'idle'
        self.joint_state = None
        self.state_time = 0.0
        self.target = None
        self.target_time = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(PoseStamped, '/vision/target_pose', self.on_target, 10)
        self.status = self.create_publisher(String, '/robot_arm/status', 10)
        self.create_timer(0.2, self.publish_status)
        self.ik = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.group)
        self.move = ActionClient(self, MoveGroup, '/move_action', callback_group=self.group)
        self.gripper = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=self.group)
        self.pick_server = ActionServer(self, PickPlace, '/robot_arm/pick_place',
            execute_callback=self.pick_place, goal_callback=self.accept_pick,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.move_server = ActionServer(self, MoveJoints, '/robot_arm/move_joints',
            execute_callback=self.move_joints, goal_callback=self.accept_move,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)

    def on_state(self, msg):
        if all(name in msg.name for name in JOINTS) and len(msg.name) == len(msg.position):
            self.joint_state = msg
            self.state_time = time.monotonic()

    def on_target(self, msg):
        self.target = msg
        self.target_time = time.monotonic()

    def publish_status(self):
        self.status.publish(String(data=json.dumps({'busy': self.motion_lock.locked(),
            'stage': self.stage, 'fault': self.fault, 'hardware_mode': self.mode,
            'gripper': 'simulated', 'position_feedback': 'commanded pulses only' if self.mode == 'gpio' else 'mock'})))

    def reserve(self):
        age = time.monotonic() - self.state_time
        if self.fault or age > 2 or not self.motion_lock.acquire(False):
            # Say why: a rejected goal otherwise reaches the client with no reason at all.
            reason = (f'fault: {self.fault}' if self.fault else
                      f'joint states are {age:.1f} s old' if age > 2 else 'another motion is running')
            self.get_logger().warning(f'Goal rejected: {reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def accept_move(self, request):
        try:
            check_targets(self.config, request.z, request.rotation)
        except ValueError:
            return GoalResponse.REJECT
        return self.reserve()

    def accept_pick(self, request):
        # No real gripper: a physical pick/place must not pretend to grasp an object.
        if self.mode != 'mock':
            return GoalResponse.REJECT
        if not all(math.isfinite(x) for x in (request.clearance, request.open_position, request.closed_position)):
            return GoalResponse.REJECT
        if not 0 < request.clearance <= 0.1 or not 0 <= request.closed_position < request.open_position <= 0.02:
            return GoalResponse.REJECT
        return self.reserve()

    def wait(self, future, seconds, parent=None):
        deadline = time.monotonic() + seconds
        while not future.done():
            if parent and parent.is_cancel_requested:
                raise Cancelled('Cancelled by operator')
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError('ROS operation timed out')
            time.sleep(0.02)
        return future.result()

    def run_action(self, client, request, parent, timeout=180):
        if parent.is_cancel_requested:
            raise Cancelled('Cancelled by operator')
        if not client.wait_for_server(timeout_sec=3):
            raise RuntimeError('Downstream action server unavailable')
        future = client.send_goal_async(request)
        try:
            child = self.wait(future, 5, parent)
        except Exception:
            # Acceptance may arrive late. Cancel it, and block new work until restart.
            self.fault = 'Uncertain downstream goal acceptance; restart after checking motion has stopped'
            def cancel_late(f):
                try:
                    goal = f.result()
                    if goal and goal.accepted:
                        goal.cancel_goal_async()
                except Exception:
                    pass
            future.add_done_callback(cancel_late)
            raise
        if not child.accepted:
            raise RuntimeError('Downstream action rejected goal')
        result_future = child.get_result_async()
        try:
            result = self.wait(result_future, timeout, parent)
        except Exception:
            try:
                self.wait(child.cancel_goal_async(), 5)
                self.wait(result_future, 5)
            except Exception:
                self.fault = 'Could not confirm downstream stop; restart after checking hardware'
            raise
        if parent.is_cancel_requested:
            raise Cancelled('Cancelled by operator')
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Downstream action failed: status={result.status}')
        return result.result

    def move_to_joints(self, positions, parent):
        check_targets(self.config, *positions)
        request = MoveGroup.Goal()
        request.request.group_name = 'arm'
        request.request.num_planning_attempts = 3
        request.request.allowed_planning_time = 5.0
        request.request.max_velocity_scaling_factor = 0.5
        request.request.max_acceleration_scaling_factor = 0.5
        request.request.start_state.is_diff = True
        constraints = Constraints()
        for name, position in zip(JOINTS, positions):
            constraints.joint_constraints.append(JointConstraint(joint_name=name, position=float(position),
                tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0))
        request.request.goal_constraints = [constraints]
        request.planning_options.plan_only = False
        request.planning_options.planning_scene_diff.is_diff = True
        request.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.run_action(self.move, request, parent)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt error {result.error_code.val}')

    def solve_pose(self, pose, parent):
        p = pose.pose
        if pose.header.frame_id != 'base_link':
            raise ValueError('Task poses must be in base_link')
        values = [p.position.x, p.position.y, p.position.z,
                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:]) - 1) > 0.002:
            raise ValueError('Pose must be finite with a unit quaternion')
        if time.monotonic() - self.state_time > 2:
            raise RuntimeError('Joint feedback is stale')
        if not self.ik.wait_for_service(timeout_sec=3):
            raise RuntimeError('/compute_ik unavailable')
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'tool0'
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        response = self.wait(self.ik.call_async(request), 5, parent)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ValueError(f'Pose unreachable or colliding (IK code {response.error_code.val})')
        state = response.solution.joint_state
        positions = [state.position[state.name.index(name)] for name in JOINTS]
        check_targets(self.config, *positions)
        return positions

    def grip(self, position, parent):
        request = GripperCommand.Goal()
        request.command.position = position
        result = self.run_action(self.gripper, request, parent, 30)
        if not result.reached_goal:
            raise RuntimeError('Simulated gripper did not reach target')

    def execute_guarded(self, goal, result, operation):
        try:
            operation()
            if goal.is_cancel_requested:
                raise Cancelled('Cancelled by operator')
            result.success, result.message = True, 'Completed'
            goal.succeed()
        except Cancelled as exc:
            result.success, result.message = False, str(exc)
            goal.canceled()
        except Exception as exc:
            result.success, result.message = False, str(exc)
            self.get_logger().error(str(exc))
            goal.abort()
        finally:
            self.stage = 'fault' if self.fault else 'idle'
            self.motion_lock.release()
        return result

    def move_joints(self, goal):
        def run():
            self.stage = 'moving'
            goal.publish_feedback(MoveJoints.Feedback(stage=self.stage))
            self.move_to_joints([goal.request.z, goal.request.rotation], goal)
        return self.execute_guarded(goal, MoveJoints.Result(), run)

    def pick_place(self, goal):
        def run():
            req = goal.request
            pick = copy.deepcopy(req.pick_pose)
            if req.use_vision_target:
                if self.target is None or time.monotonic() - self.target_time > 1:
                    raise ValueError('No fresh vision target')
                age = (self.get_clock().now().nanoseconds -
                       (self.target.header.stamp.sec * 10**9 + self.target.header.stamp.nanosec)) / 1e9
                if not -0.1 <= age <= 1.0:
                    raise ValueError('Vision timestamp is stale or in the future')
                pick = copy.deepcopy(self.target)
            place = copy.deepcopy(req.place_pose)
            above_pick, above_place = copy.deepcopy(pick), copy.deepcopy(place)
            above_pick.pose.position.z += req.clearance
            above_place.pose.position.z += req.clearance
            poses = [above_pick, pick, above_place, place]
            # Check all endpoints before opening the gripper or moving.
            joints = [self.solve_pose(pose, goal) for pose in poses]
            steps = [('open', lambda: self.grip(req.open_position, goal)),
                     ('approach_pick', lambda: self.move_to_joints(joints[0], goal)),
                     ('lower_pick', lambda: self.move_to_joints(joints[1], goal)),
                     ('close', lambda: self.grip(req.closed_position, goal)),
                     ('lift', lambda: self.move_to_joints(joints[0], goal)),
                     ('transfer', lambda: self.move_to_joints(joints[2], goal)),
                     ('lower_place', lambda: self.move_to_joints(joints[3], goal)),
                     ('release', lambda: self.grip(req.open_position, goal)),
                     ('retreat', lambda: self.move_to_joints(joints[2], goal))]
            for index, (stage, operation) in enumerate(steps):
                self.stage = stage
                goal.publish_feedback(PickPlace.Feedback(stage=stage, progress=index / len(steps)))
                operation()
            goal.publish_feedback(PickPlace.Feedback(stage='complete', progress=1.0))
        return self.execute_guarded(goal, PickPlace.Result(), run)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
