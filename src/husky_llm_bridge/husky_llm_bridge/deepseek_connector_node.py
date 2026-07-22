#!/usr/bin/env python3
import json
import os
import urllib.request
import urllib.error
import yaml
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from husky_msgs.msg import FleetState
from husky_llm_bridge.fleet_state_util import format_fleet_state
from husky_llm_bridge.waypoint_loader import WaypointLoader


class DeepSeekConnectorNode(Node):
    def __init__(self):
        super().__init__('deepseek_connector_node')

        self.declare_parameter('api_key', '')
        self.declare_parameter('base_url', 'https://opencode.ai/zen/v1/chat/completions')
        self.declare_parameter('model', 'deepseek-v4-flash')
        self.declare_parameter('fleet_config_path', '')
        self.declare_parameter('waypoints_config_path', '')

        self.api_key = self.get_parameter('api_key').value or os.environ.get('OPENCODE_GO_API_KEY', '')
        self.api_key_from_env = not self.api_key
        self.base_url = self.get_parameter('base_url').value
        self.model = self.get_parameter('model').value
        self.valid_robots = self._load_fleet_config(self.get_parameter('fleet_config_path').value)
        self.waypoint_loader = WaypointLoader(self.get_parameter('waypoints_config_path').value)

        if not self.api_key:
            self.get_logger().warn('No OPENCODE_GO_API_KEY set. Connector will fail on first command.')

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

        self.latest_fleet_state = None
        self.last_command = None
        self.robot_poses = {}
        for robot_id in self.valid_robots:
            odom_topic = f'/{robot_id}/platform/odom/filtered'
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.get_logger().info(f'Subscribed to odometry for {robot_id} on {odom_topic}')
        self.get_logger().info(f'OpenCode Go connector initialized. URL: {self.base_url}, Model: {self.model}')

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

    def command_callback(self, msg: String):
        command = msg.data
        if command == self.last_command:
            return
        self.get_logger().info(f'Received command: {command}')
        self.last_command = command

        if self.api_key_from_env:
            self.api_key = os.environ.get('OPENCODE_GO_API_KEY', '')

        if not self.api_key:
            self._publish_status('No OPENCODE_GO_API_KEY configured')
            return

        fleet_state_json = format_fleet_state(self.latest_fleet_state)
        prompt = self._build_prompt(command, fleet_state_json)

        try:
            response = self._call_deepseek(prompt)
            self.raw_decision_pub.publish(self._make_string(response))
            self.get_logger().info('DeepSeek response published to /llm/raw_decision')
        except Exception as e:
            self._publish_status(f'DeepSeek API error: {e}')

    def _build_prompt(self, command, fleet_state):
        valid_robots_str = ', '.join(self.valid_robots) if self.valid_robots else 'unknown'
        primary_robot = self.valid_robots[0] if self.valid_robots else 'unknown'
        waypoint_names = self.waypoint_loader.get_available_names()
        waypoint_names_str = ', '.join(waypoint_names) if waypoint_names else 'none'

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
            f"Available waypoint names: {waypoint_names_str}\n\n"
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

    def _call_deepseek(self, prompt):
        payload = json.dumps({
            'model': self.model,
            'messages': [
                {'role': 'user', 'content': '/no_think\n' + prompt}
            ],
            'stream': False,
            'temperature': 0.0,
        }).encode('utf-8')

        req = urllib.request.Request(
            self.base_url,
            data=payload,
            headers={
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {self.api_key}'
            },
            method='POST'
        )

        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        choices = body.get('choices', [])
        if not choices:
            raise RuntimeError('No choices in OpenCode Go response')

        message = choices[0].get('message', {})
        content = message.get('content', '')
        if not content:
            raise RuntimeError('No content in OpenCode Go response')

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
    node = DeepSeekConnectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
