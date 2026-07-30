#!/usr/bin/env python3
import json
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SYSTEM_PROMPT = """You are a fleet mission planner for outdoor Husky A200 robots.

Current fleet state:
{fleet_state}

User command:
{command}

You MUST respond with ONLY a JSON object in this exact format. No extra text, no markdown fences.

{{
  "missions": [
    {{
      "action": "navigate",
      "robot_id": "<robot namespace>",
      "waypoints": [{{"x": <float>, "y": <float>}}],
      "priority": <int 1-5>
    }},
    {{
      "action": "set_state",
      "robot_id": "<robot namespace>",
      "waiting": <bool>,
      "avoidance_enabled": <bool>
    }}
  ]
}}

Rules:
- action must be "navigate" or "set_state"
- robot_id must be one of: {valid_robots}
- navigate: waypoints[] required, each with numeric x and y. priority 1-5 (optional, default 3).
- set_state: at least one of waiting or avoidance_enabled as boolean.
- Return the JSON object only. No explanation, no markdown."""


class LLMValidatorNode(Node):
    def __init__(self):
        super().__init__('llm_validator_node')

        self.declare_parameter('fleet_config_path', '')
        self.declare_parameter('waypoints_config_path', '')

        fleet_config_path = self.get_parameter('fleet_config_path').value
        self.valid_robots = self._load_fleet_config(fleet_config_path)

        waypoints_config_path = self.get_parameter('waypoints_config_path').value
        self.valid_waypoint_names = self._load_waypoint_names(waypoints_config_path)

        self.raw_decision_sub = self.create_subscription(
            String,
            '/llm/raw_decision',
            self.raw_decision_callback,
            10
        )

        self.decision_pub = self.create_publisher(String, '/llm/decision', 10)
        self.status_pub = self.create_publisher(String, '/llm/decision_status', 10)
        self.natural_language_pub = self.create_publisher(String, '/llm/natural_language_response', 10)

        self.get_logger().info(f'LLM validator initialized. Valid robots: {self.valid_robots}, Valid waypoints: {self.valid_waypoint_names}')

    def _load_fleet_config(self, path):
        if not path:
            self.get_logger().warn('No fleet_config_path provided, validator will reject all missions')
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

    def _load_waypoint_names(self, path):
        if not path:
            return set()
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            names = set()
            for key, wp in config.get('waypoints', {}).items():
                names.add(key)
                display_name = wp.get('name', '')
                if display_name:
                    names.add(display_name)
            return names
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints config: {e}')
            return set()

    def raw_decision_callback(self, msg: String):
        raw = msg.data.strip()
        if raw.startswith('```'):
            first_newline = raw.find('\n')
            if first_newline != -1:
                raw = raw[first_newline + 1:]
            else:
                raw = raw[3:]
            if raw.endswith('```'):
                raw = raw[:-3].strip()

        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            # Not JSON — treat as natural language response (e.g., answering a question)
            self.natural_language_pub.publish(msg)
            self.get_logger().info('Natural language response forwarded')
            return

        error = self._validate(data)
        if error:
            self._publish_status(False, error)
            return

        self.decision_pub.publish(msg)
        self.get_logger().info('Decision validated and forwarded')

    def _validate(self, data):
        if not isinstance(data, dict):
            return 'Top-level must be an object'

        missions = data.get('missions')
        if missions is None:
            return 'Missing "missions" key'
        if not isinstance(missions, list):
            return '"missions" must be an array'
        if len(missions) == 0:
            return '"missions" array is empty'

        for i, mission in enumerate(missions):
            error = self._validate_mission(i, mission)
            if error:
                return error

        return None

    def _validate_mission(self, idx, mission):
        prefix = f'Mission[{idx}]'

        if not isinstance(mission, dict):
            return f'{prefix}: must be an object'

        action = mission.get('action')
        if action not in ('navigate', 'set_state', 'rotate', 'emergency_stop', 'go_home', 'clear_emergency_stop'):
            return f'{prefix}: action must be "navigate", "set_state", "rotate", "emergency_stop", "go_home", or "clear_emergency_stop", got "{action}"'

        robot_id = mission.get('robot_id')
        if not isinstance(robot_id, str) or not robot_id:
            return f'{prefix}: missing or invalid "robot_id"'
        if robot_id not in self.valid_robots:
            return f'{prefix}: robot_id "{robot_id}" not in fleet config'

        if action == 'navigate':
            return self._validate_navigate(idx, mission)
        elif action == 'set_state':
            return self._validate_set_state(idx, mission)
        elif action == 'rotate':
            return self._validate_rotate(idx, mission)
        elif action == 'emergency_stop':
            return self._validate_emergency_stop(idx, mission)
        elif action == 'go_home':
            return self._validate_go_home(idx, mission)
        elif action == 'clear_emergency_stop':
            return self._validate_clear_emergency_stop(idx, mission)

        return None

    def _validate_navigate(self, idx, mission):
        prefix = f'Mission[{idx}]'

        waypoints = mission.get('waypoints')
        waypoint_name = mission.get('waypoint_name')
        waypoint_names = mission.get('waypoint_names')
        relative = mission.get('relative')

        if waypoints is None and waypoint_name is None and waypoint_names is None and relative is None:
            return f'{prefix}: must have "waypoints", "waypoint_name", "waypoint_names", or "relative"'

        if waypoints is not None:
            if not isinstance(waypoints, list) or len(waypoints) == 0:
                return f'{prefix}: "waypoints" must be a non-empty array'

            for j, wp in enumerate(waypoints):
                if not isinstance(wp, dict):
                    return f'{prefix}: waypoint[{j}] must be an object'
                x = wp.get('x')
                y = wp.get('y')
                lat = wp.get('lat')
                lon = wp.get('lon')

                has_odom = x is not None and y is not None
                has_gps = lat is not None and lon is not None

                if not has_odom and not has_gps:
                    return f'{prefix}: waypoint[{j}] must have {x, y} or {lat, lon}'

                if has_odom:
                    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                        return f'{prefix}: waypoint[{j}] x and y must be numeric'

                if has_gps:
                    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
                        return f'{prefix}: waypoint[{j}] lat and lon must be numeric'
                    if lat < -90 or lat > 90:
                        return f'{prefix}: waypoint[{j}] lat must be between -90 and 90'
                    if lon < -180 or lon > 180:
                        return f'{prefix}: waypoint[{j}] lon must be between -180 and 180'

        if relative is not None:
            if not isinstance(relative, dict):
                return f'{prefix}: "relative" must be an object'
            forward = relative.get('forward', 0.0)
            right = relative.get('right', 0.0)
            if not isinstance(forward, (int, float)):
                return f'{prefix}: relative.forward must be numeric'
            if not isinstance(right, (int, float)):
                return f'{prefix}: relative.right must be numeric'

        if waypoint_name is not None:
            if not isinstance(waypoint_name, str) or not waypoint_name:
                return f'{prefix}: "waypoint_name" must be a non-empty string'
            if self.valid_waypoint_names and waypoint_name not in self.valid_waypoint_names:
                return f'{prefix}: "waypoint_name" "{waypoint_name}" not in available waypoints: {sorted(self.valid_waypoint_names)}'

        if waypoint_names is not None:
            if not isinstance(waypoint_names, list) or len(waypoint_names) == 0:
                return f'{prefix}: "waypoint_names" must be a non-empty array'
            for j, name in enumerate(waypoint_names):
                if not isinstance(name, str) or not name:
                    return f'{prefix}: waypoint_names[{j}] must be a non-empty string'
                if self.valid_waypoint_names and name not in self.valid_waypoint_names:
                    return f'{prefix}: waypoint_names[{j}] "{name}" not in available waypoints: {sorted(self.valid_waypoint_names)}'

        priority = mission.get('priority')
        if priority is not None:
            if not isinstance(priority, (int, float)) or isinstance(priority, bool) or priority < 1 or priority > 5:
                return f'{prefix}: priority must be numeric 1-5, got {priority}'

        return None

    def _validate_set_state(self, idx, mission):
        prefix = f'Mission[{idx}]'

        waiting = mission.get('waiting')
        avoidance = mission.get('avoidance_enabled')

        if waiting is None and avoidance is None:
            return f'{prefix}: set_state must have at least one of "waiting" or "avoidance_enabled"'

        if waiting is not None and not isinstance(waiting, bool):
            return f'{prefix}: "waiting" must be a boolean'
        if avoidance is not None and not isinstance(avoidance, bool):
            return f'{prefix}: "avoidance_enabled" must be a boolean'

        return None

    def _validate_rotate(self, idx, mission):
        prefix = f'Mission[{idx}]'

        angle_deg = mission.get('angle_deg')
        if angle_deg is None:
            return f'{prefix}: missing "angle_deg"'
        if not isinstance(angle_deg, (int, float)):
            return f'{prefix}: "angle_deg" must be numeric'
        if angle_deg < -360 or angle_deg > 360:
            return f'{prefix}: "angle_deg" must be between -360 and 360'

        return None

    def _validate_emergency_stop(self, idx, mission):
        return None

    def _validate_go_home(self, idx, mission):
        return None

    def _validate_clear_emergency_stop(self, idx, mission):
        return None

    def _publish_status(self, success, reason):
        status = {'success': success, 'reason': reason}
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        if not success:
            self.get_logger().warn(f'Validation failed: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
