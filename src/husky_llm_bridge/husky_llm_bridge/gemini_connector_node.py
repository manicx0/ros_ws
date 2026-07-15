#!/usr/bin/env python3
import json
import os
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from husky_msgs.msg import FleetState
from husky_llm_bridge.fleet_state_util import format_fleet_state


class GeminiConnectorNode(Node):
    def __init__(self):
        super().__init__('gemini_connector_node')

        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'gemini-3.5-flash')

        self.api_key = self.get_parameter('api_key').value or os.environ.get('GEMINI_API_KEY', '')
        self.api_key_from_env = not self.api_key
        self.model = self.get_parameter('model').value

        if not self.api_key:
            self.get_logger().warn('No GEMINI_API_KEY set. Connector will fail on first command.')

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
        self.get_logger().info(f'Gemini connector initialized. Model: {self.model}')

    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg

    def command_callback(self, msg: String):
        command = msg.data
        if command == self.last_command:
            return
        self.get_logger().info(f'Received command: {command}')
        self.last_command = command

        if self.api_key_from_env:
            self.api_key = os.environ.get('GEMINI_API_KEY', '')

        if not self.api_key:
            self._publish_status('No GEMINI_API_KEY configured')
            return

        fleet_state_json = format_fleet_state(self.latest_fleet_state)
        prompt = self._build_prompt(command, fleet_state_json)

        try:
            response = self._call_gemini(prompt)
            self.raw_decision_pub.publish(self._make_string(response))
            self.get_logger().info('Gemini response published to /llm/raw_decision')
        except Exception as e:
            self._publish_status(f'Gemini API error: {e}')

    def _build_prompt(self, command, fleet_state):
        return (
            "You are a fleet mission planner for outdoor Husky A200 robots.\n\n"
            f"Current fleet state:\n{fleet_state}\n\n"
            f"User command:\n{command}\n\n"
            'You MUST respond with ONLY a JSON object in this exact format. '
            'No extra text, no markdown fences.\n\n'
            '{\n'
            '  "missions": [\n'
            '    {\n'
            '      "action": "navigate",\n'
            '      "robot_id": "<robot namespace>",\n'
            '      "waypoints": [{"x": <float>, "y": <float>}],\n'
            '      "priority": <int 1-5>\n'
            '    },\n'
            '    {\n'
            '      "action": "set_state",\n'
            '      "robot_id": "<robot namespace>",\n'
            '      "waiting": <bool>,\n'
            '      "avoidance_enabled": <bool>\n'
            '    }\n'
            '  ]\n'
            '}\n\n'
            'Rules:\n'
            '- action must be "navigate" or "set_state"\n'
            '- navigate: waypoints[] required, each with numeric x and y. priority 1-5 (optional, default 3).\n'
            '- set_state: at least one of waiting or avoidance_enabled as boolean.\n'
            '- Return the JSON object only. No explanation, no markdown.'
        )

    def _call_gemini(self, prompt):
        url = f'https://generativelanguage.googleapis.com/v1beta/models/{self.model}:generateContent?key={self.api_key}'

        payload = json.dumps({
            'contents': [{'parts': [{'text': prompt}]}],
        }).encode('utf-8')

        req = urllib.request.Request(
            url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST'
        )

        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        candidates = body.get('candidates', [])
        if not candidates:
            raise RuntimeError('No candidates in Gemini response')

        parts = candidates[0].get('content', {}).get('parts', [])
        if not parts:
            raise RuntimeError('No parts in Gemini response')

        text = parts[0].get('text', '').strip()
        return text

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
    node = GeminiConnectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
