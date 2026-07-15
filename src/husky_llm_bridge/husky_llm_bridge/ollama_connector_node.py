#!/usr/bin/env python3
import json
import urllib.request
import urllib.error
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from husky_msgs.msg import FleetState
from husky_llm_bridge.fleet_state_util import format_fleet_state


class OllamaConnectorNode(Node):
    def __init__(self):
        super().__init__('ollama_connector_node')

        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('model', 'llama3.2')

        self.ollama_url = self.get_parameter('ollama_url').value
        self.model = self.get_parameter('model').value

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
        self.get_logger().info(f'Ollama connector initialized. URL: {self.ollama_url}, Model: {self.model}')

    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg

    def command_callback(self, msg: String):
        command = msg.data
        if command == self.last_command:
            return
        self.get_logger().info(f'Received command: {command}')
        self.last_command = command

        fleet_state_json = format_fleet_state(self.latest_fleet_state)
        prompt = self._build_prompt(command, fleet_state_json)

        try:
            response = self._call_ollama(prompt)
            self.raw_decision_pub.publish(self._make_string(response))
            self.get_logger().info('Ollama response published to /llm/raw_decision')
        except Exception as e:
            self._publish_status(f'Ollama API error: {e}')

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

    def _call_ollama(self, prompt):
        url = f'{self.ollama_url}/api/generate'

        payload = json.dumps({
            'model': self.model,
            'prompt': prompt,
            'stream': False,
            'format': 'json',
        }).encode('utf-8')

        req = urllib.request.Request(
            url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST'
        )

        with urllib.request.urlopen(req, timeout=60) as resp:
            body = json.loads(resp.read().decode('utf-8'))

        response_text = body.get('response', '').strip()
        return response_text

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
