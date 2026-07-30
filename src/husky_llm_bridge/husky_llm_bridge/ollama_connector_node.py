#!/usr/bin/env python3
import json
import math
import time
import uuid
import queue
import threading
import urllib.request
import urllib.error
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from husky_msgs.msg import FleetState
from husky_llm_bridge.fleet_state_util import format_fleet_state
from husky_llm_bridge.waypoint_loader import WaypointLoader


class OllamaConnectorNode(Node):
    def __init__(self):
        super().__init__('ollama_connector_node')

        self.declare_parameter('ollama_url', 'http://host.docker.internal:11434')
        self.declare_parameter('model', 'qwen3.5-abliterated')
        self.declare_parameter('fleet_config_path', '')
        self.declare_parameter('waypoints_config_path', '')

        self.ollama_url = self.get_parameter('ollama_url').value
        self.model = self.get_parameter('model').value or 'qwen3.5-abliterated'
        self.valid_robots = self._load_fleet_config(self.get_parameter('fleet_config_path').value)
        self.waypoint_loader = WaypointLoader(self.get_parameter('waypoints_config_path').value)

        self.command_sub = self.create_subscription(
            String,
            '/llm/command',
            self.command_callback,
            10
        )

        self.fleet_state_sub = self.create_subscription(
            FleetState,
            '/fleet/robot_states',
            self.fleet_state_callback,
            10
        )

        self.raw_decision_pub = self.create_publisher(String, '/llm/raw_decision', 10)
        self.status_pub = self.create_publisher(String, '/llm/decision_status', 10)
        self.command_status_pub = self.create_publisher(String, '/llm/command_status', 10)

        self.latest_fleet_state = None
        self._last_command_id = None
        self._last_command_text = None
        self._last_command_time = 0.0
        self.robot_poses = {}
        for robot_id in self.valid_robots:
            odom_topic = f'/{robot_id}/platform/odom'
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.get_logger().info(f'Subscribed to odometry for {robot_id} on {odom_topic}')

        self._mission_status_sub = self.create_subscription(
            String,
            '/llm/mission_status',
            self._mission_status_callback,
            10
        )

        self._command_queue = queue.Queue()
        self._worker_thread = threading.Thread(target=self._process_queue, daemon=True)
        self._worker_thread.start()

        self.get_logger().info('Waiting for initial odometry...')
        for robot_id in self.valid_robots:
            if robot_id not in self.robot_poses:
                for _ in range(50):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if robot_id in self.robot_poses:
                        break
                if robot_id in self.robot_poses:
                    self.get_logger().info(f'Odometry received for {robot_id}')
                else:
                    self.get_logger().warn(f'No odometry from {robot_id} after 5s')

        self.get_logger().info(f'Ollama connector initialized. URL: {self.ollama_url}, Model: {self.model}')

        self.create_timer(2.0, self._warmup_once)

    def _warmup_once(self):
        self.get_logger().info('Warming up model...')
        try:
            self._call_ollama('say hello')
            self.get_logger().info('Model warmup complete')
        except Exception as e:
            self.get_logger().warn(f'Model warmup failed (non-fatal): {e}')

    def _publish_command_status(self, command_id, status, reason=''):
        msg = String()
        msg.data = json.dumps({
            'command_id': command_id,
            'status': status,
            'reason': reason
        })
        self.command_status_pub.publish(msg)

    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg

    def odom_callback(self, msg: Odometry, robot_id: str):
        pose = msg.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.robot_poses[robot_id] = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw
        }

    def _load_fleet_config(self, path):
        if not path:
            return []
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            robots = []
            for entry in config.get('fleet_manager', {}).get('robots', []):
                ns = entry.get('namespace', '')
                if ns:
                    robots.append(ns)
            return robots
        except Exception as e:
            self.get_logger().error(f'Failed to load fleet config: {e}')
            return []

    def _mission_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = data.get('state', '')
        if state in ('SUCCEEDED', 'EXECUTED_FAILED'):
            self._last_command_text = None
            self._last_command_time = 0.0
            self.get_logger().info('Mission completed — cleared dedup cache')

    def command_callback(self, msg: String):
        command = msg.data.strip()
        if not command:
            return
        command_id = str(uuid.uuid4())
        elapsed = time.time() - self._last_command_time if self._last_command_time else 999.0
        if command == self._last_command_text and elapsed < 30.0:
            if self._last_command_id:
                self._publish_command_status(
                    command_id, 'skipped',
                    'Duplicate command — already being processed (id: ' + self._last_command_id + ')')
            return
        self.get_logger().info(f'[{command_id[:8]}] Received command: {command}')
        self._last_command_id = command_id
        self._last_command_text = command
        self._last_command_time = time.time()
        self._publish_command_status(command_id, 'queued')
        self._command_queue.put((command_id, command))

    def _process_queue(self):
        while rclpy.ok():
            try:
                command_id, command = self._command_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            self._publish_command_status(command_id, 'planning')
            self.get_logger().info(f'[{command_id[:8]}] Processing command: {command}')

            fleet_state_json = format_fleet_state(self.latest_fleet_state)
            prompt = self._build_prompt(command, fleet_state_json)

            try:
                response = self._call_ollama(prompt)
                self.raw_decision_pub.publish(self._make_string(response))
                self._publish_command_status(command_id, 'done')
                self.get_logger().info(f'[{command_id[:8]}] Ollama response published')
            except Exception as e:
                reason = f'Ollama API error: {e}'
                self._publish_command_status(command_id, 'failed', reason)
                self._publish_status(reason)

    def _build_prompt(self, command, fleet_state):
        valid_robots_str = ', '.join(self.valid_robots) if self.valid_robots else 'unknown'
        primary_robot = self.valid_robots[0] if self.valid_robots else 'unknown'
        waypoints_str = self.waypoint_loader.get_formatted_waypoints()

        odom_info = []
        for robot_id in self.valid_robots:
            pose = self.robot_poses.get(robot_id)
            if pose:
                odom_info.append(
                    f'{robot_id}: x={pose["x"]:.2f}, y={pose["y"]:.2f}, yaw={pose["yaw"]:.2f} rad'
                )
            else:
                odom_info.append(f'{robot_id}: no odometry data yet')
        odom_str = '\n'.join(odom_info)
        
        return (
            "You are a fleet mission planner for outdoor Husky A200 robots.\n\n"
            f"Available robot IDs: {primary_robot} (primary), {', '.join(self.valid_robots[1:]) if len(self.valid_robots) > 1 else ''}\n\n"
            f"Available waypoints:\n{waypoints_str}\n\n"
            f"Current fleet state:\n{fleet_state}\n\n"
            f"Current robot positions (odometry):\n{odom_str}\n\n"
            f"User command:\n{command}\n\n"
            "Respond based on what the user is asking:\n\n"
            "1. If the user is asking a QUESTION (e.g., 'is the robot stuck?', 'what's the status?'), "
            "respond with a natural language answer based on the fleet state.\n\n"
            "2. If the user is giving a COMMAND (e.g., 'send robot to x=3,y=0', 'go to point_a', 'stop all robots', 'turn 180 degrees'), "
            "respond with ONLY a JSON object in this exact format. No extra text, no markdown fences.\n\n"
            '{\n'
            '  "missions": [\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "waypoints": [{"x": <float>, "y": <float>}],\n'
            '      "priority": <int 1-5>\n'
            '    },\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "waypoints": [{"x": <float>, "y": <float>, "yaw": <float>}]\n'
            '    },\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "waypoint_name": "<name from available waypoint names>"\n'
            '    },\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "waypoints": [{"lat": <float>, "lon": <float>}]\n'
            '    },\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "relative": {"forward": <float>, "right": <float>}\n'
            '    },\n'
            '    {\n'
            '      "action": "set_state",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "waiting": <bool>,\n'
            '      "avoidance_enabled": <bool>\n'
            '    },\n'
            '    {\n'
            '      "action": "rotate",\n'
            '      "robot_id": "<robot namespace from available list>",\n'
            '      "angle_deg": <float>\n'
            '    },\n'
            '    {\n'
            '      "action": "emergency_stop",\n'
            '      "robot_id": "<robot namespace from available list>"\n'
            '    },\n'
            '    {\n'
            '      "action": "clear_emergency_stop",\n'
            '      "robot_id": "<robot namespace from available list>"\n'
            '    },\n'
            '    {\n'
            '      "action": "go_home",\n'
            '      "robot_id": "<robot namespace from available list>"\n'
            '    }\n'
            '  ]\n'
            '}\n\n'
            'Rules for JSON missions:\n'
            '- action must be "navigate", "set_state", "rotate", "emergency_stop", "clear_emergency_stop", or "go_home"\n'
            '- robot_id MUST be one of the available robot IDs listed above\n'
            '- If the user says "the robot", "send robot", or doesn\'t specify which robot, use the primary robot\n'
            '- navigate: provide one of: "waypoints" [{x,y} or {x,y,yaw} or {lat,lon}], "waypoint_name", or "relative" {"forward": meters, "right": meters}. yaw is optional (radians, robot faces that direction at goal). priority 1-5 (optional, default 3).\n'
            '- "relative" moves: use for time/distance commands like "move forward 2 meters" or "go forward for 3 seconds". The bridge computes absolute waypoints from odometry. Do NOT use "waypoints" with computed coordinates for time/distance commands — use "relative" instead with forward/right in meters. At 0.45 m/s, 3 seconds = 1.35 meters forward.\n'
            '- set_state: at least one of waiting or avoidance_enabled as boolean.\n'
            '- rotate: angle_deg required (float, degrees, positive=CCW, negative=CW). Use for "turn", "rotate", "spin" commands.\n'
            '- emergency_stop: immediately stops the robot. Use for "stop", "halt", "emergency stop", "abort" commands.\n'
            '- clear_emergency_stop: clears emergency stop flag after it was triggered. Use for "resume", "clear stop", "continue" commands after emergency stop.\n'
            '- go_home: sends robot to its charging station / origin. Use for "go home", "return to base", "return to origin", "go back to start" commands.\n'
            '- Return the JSON object only. No explanation, no markdown.'
        )

    def _call_ollama(self, prompt):
        url = f'{self.ollama_url}/v1/chat/completions'

        payload = json.dumps({
            'model': self.model,
            'messages': [
                {'role': 'user', 'content': '/no_think\n' + prompt}
            ],
            'stream': False,
            'temperature': 0.0,
            'options': {'num_ctx': 4096},
        }).encode('utf-8')

        req = urllib.request.Request(
            url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST'
        )

        with urllib.request.urlopen(req, timeout=120) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        choices = body.get('choices', [])
        if not choices:
            raise RuntimeError('No choices in Ollama response')

        message = choices[0].get('message', {})
        content = message.get('content', '')
        if not content:
            raise RuntimeError('No content in Ollama response')

        return content.strip()

    def _publish_status(self, reason):
        status = {'success': False, 'reason': reason}
        self.status_pub.publish(self._make_string(json.dumps(status)))
        self.get_logger().warn(f'Status: {reason}')

    def _make_string(self, data):
        msg = String()
        msg.data = data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = OllamaConnectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
