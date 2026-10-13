import copy
import json
from pathlib import Path
import secrets
import threading
import time
from flask import Flask, jsonify, request, Response
from werkzeug.serving import make_server
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from roboarm_interfaces.action import MoveJoints, PickPlace
from .configuration import load, save, validate, check_targets, cylindrical_pose, finite


class Web(Node):
    def __init__(self):
        super().__init__('web_ui')
        for name, value in {'host': '127.0.0.1', 'port': 5000, 'hardware_config': '', 'hardware_mode': 'mock'}.items():
            self.declare_parameter(name, value)
        self.path = self.get_parameter('hardware_config').value
        self.active_config = load(self.path)
        self.mode = self.get_parameter('hardware_mode').value
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.job = None
        self.handle = None
        self.task_status = {}
        self.status_time = 0.0
        self.positions = {}
        self.token = secrets.token_urlsafe(32)
        self.move = ActionClient(self, MoveJoints, '/robot_arm/move_joints', callback_group=self.group)
        self.pick = ActionClient(self, PickPlace, '/robot_arm/pick_place', callback_group=self.group)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=self.group)
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller', callback_group=self.group)
        self.create_subscription(String, '/robot_arm/status', self.on_status, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.app = Flask(__name__)
        self.app.config['MAX_CONTENT_LENGTH'] = 32768
        self.routes()

    def on_status(self, msg):
        with self.lock:
            self.task_status = json.loads(msg.data)
            self.status_time = time.monotonic()

    def on_joints(self, msg):
        with self.lock:
            self.positions = dict(zip(msg.name, msg.position))

    def require_idle(self):
        if time.monotonic() - self.status_time > 2:
            raise ValueError('Task manager status is stale')
        if self.task_status.get('fault'):
            raise ValueError(self.task_status['fault'])
        if self.task_status.get('busy') or (self.job and self.job.get('state') in ('sending', 'running', 'canceling', 'unknown')):
            raise ValueError('Wait for motion to finish or cancel it first')

    def submit(self, client, goal):
        with self.lock:
            self.require_idle()
            if not client.server_is_ready():
                raise ValueError('Task action server is unavailable')
            self.job = {'state': 'sending', 'message': '', 'feedback': ''}
            self.handle = None
            future = client.send_goal_async(goal, feedback_callback=self.feedback)
            future.add_done_callback(self.accepted)
            return dict(self.job)

    def feedback(self, msg):
        with self.lock:
            if self.job:
                self.job['feedback'] = msg.feedback.stage

    def accepted(self, future):
        with self.lock:
            try:
                self.handle = future.result()
                if not self.handle.accepted:
                    self.job.update(state='rejected', message='Task rejected: check limits, mode and fresh joint states')
                    return
                cancel_requested = self.job['state'] == 'canceling'
                self.job['state'] = 'canceling' if cancel_requested else 'running'
                if cancel_requested:
                    self.handle.cancel_goal_async()
                self.handle.get_result_async().add_done_callback(self.finished)
            except Exception as exc:
                self.job.update(state='unknown', message=str(exc))

    def finished(self, future):
        with self.lock:
            try:
                result = future.result()
                self.job.update(state='done', success=result.result.success, message=result.result.message,
                                action_status=result.status)
                self.handle = None
            except Exception as exc:
                self.job.update(state='unknown', message=str(exc))

    @staticmethod
    def wait(future, timeout=5):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise TimeoutError('Service result not confirmed; inspect controller state before retrying')
        return future.result()

    def pose(self, z, rotation):
        check_targets(self.active_config, z, rotation)
        x, y, height, qz, qw = cylindrical_pose(self.active_config['arm_radius'],
            self.active_config['base_height'], z, rotation)
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, height
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        return pose

    def routes(self):
        app = self.app

        @app.before_request
        def guard():
            if request.method != 'GET':
                if not secrets.compare_digest(request.headers.get('X-CSRF-Token', ''), self.token):
                    return jsonify(error='Reload the page before issuing commands'), 403
                if not request.is_json:
                    return jsonify(error='JSON body required'), 400

        @app.errorhandler(ValueError)
        @app.errorhandler(KeyError)
        @app.errorhandler(TypeError)
        def bad_request(exc):
            return jsonify(error=str(exc)), 400

        @app.errorhandler(TimeoutError)
        def timeout(exc):
            return jsonify(error=str(exc)), 504

        @app.get('/')
        def index():
            html = Path(__file__).with_name('web.html').read_text(encoding='utf-8')
            return Response(html.replace('__TOKEN__', self.token), mimetype='text/html',
                            headers={'Cache-Control': 'no-store'})

        @app.get('/api/status')
        def status():
            with self.lock:
                return jsonify(task=self.task_status, job=self.job, joints=self.positions,
                    status_fresh=time.monotonic() - self.status_time < 2, hardware_mode=self.mode,
                    gripper='simulated')

        @app.get('/api/configuration')
        def configuration():
            return jsonify(active=self.active_config, saved=load(self.path), path=self.path,
                           apply='Restart bringup to apply saved changes')

        @app.post('/api/configuration')
        def configure():
            with self.lock:
                self.require_idle()
                data = validate(request.get_json())
                save(self.path, data)
            return jsonify(message='Saved. Active limits are unchanged until bringup restarts.')

        @app.post('/api/move')
        def move():
            data = request.get_json()
            z, rotation = finite(data['z'], 'z'), finite(data['rotation'], 'rotation')
            check_targets(self.active_config, z, rotation)
            return jsonify(self.submit(self.move, MoveJoints.Goal(z=z, rotation=rotation))), 202

        @app.post('/api/pick_place')
        def pick_place():
            if self.mode != 'mock':
                raise ValueError('Pick/place is simulation only until a physical gripper is installed')
            data = request.get_json()
            goal = PickPlace.Goal()
            goal.pick_pose = self.pose(finite(data['pick_z'], 'pick_z'), finite(data['pick_rotation'], 'pick_rotation'))
            goal.place_pose = self.pose(finite(data['place_z'], 'place_z'), finite(data['place_rotation'], 'place_rotation'))
            goal.clearance = finite(data.get('clearance', 0.02), 'clearance')
            goal.open_position, goal.closed_position = 0.01, 0.0
            if not isinstance(data.get('use_vision', False), bool):
                raise ValueError('use_vision must be boolean')
            goal.use_vision_target = data.get('use_vision', False)
            return jsonify(self.submit(self.pick, goal)), 202

        @app.post('/api/cancel')
        def cancel():
            with self.lock:
                if not self.job or self.job['state'] not in ('sending', 'running', 'canceling'):
                    return jsonify(message='No Web UI goal is active')
                self.job['state'] = 'canceling'
                if self.handle:
                    self.handle.cancel_goal_async()
            return jsonify(message='Cancellation requested; wait for terminal result'), 202

        @app.get('/api/controllers')
        def controllers():
            if not self.list_client.service_is_ready():
                raise ValueError('Controller manager unavailable')
            reply = self.wait(self.list_client.call_async(ListControllers.Request()))
            return jsonify(controllers=[{'name': c.name, 'state': c.state, 'type': c.type} for c in reply.controller])

        @app.post('/api/controllers/switch')
        def switch():
            with self.lock:
                self.require_idle()
                data = request.get_json()
                name = data['name']
                if name not in ('arm_controller', 'gripper_controller') or not isinstance(data['activate'], bool):
                    raise ValueError('Select arm_controller or gripper_controller and a boolean activate value')
                if not self.switch_client.service_is_ready():
                    raise ValueError('Controller manager unavailable')
                req = SwitchController.Request()
                req.activate_controllers = [name] if data['activate'] else []
                req.deactivate_controllers = [] if data['activate'] else [name]
                req.strictness = SwitchController.Request.STRICT
                req.activate_asap = True
                req.timeout.sec = 3
                # Do not hold the data lock while waiting for callbacks on the ROS executor.
                future = self.switch_client.call_async(req)
            return jsonify(ok=self.wait(future).ok)


def main(args=None):
    rclpy.init(args=args)
    node = Web()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    server = make_server(node.get_parameter('host').value, node.get_parameter('port').value, node.app, threaded=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f'Web UI: http://{node.get_parameter("host").value}:{node.get_parameter("port").value}')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
