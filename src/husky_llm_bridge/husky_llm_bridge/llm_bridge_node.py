#!/usr/bin/env python3
import json
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry
from husky_msgs.msg import FleetState, FleetGoal, GoalEvent
from husky_msgs.action import FleetNavigate
from husky_msgs.srv import FleetSetState
from husky_llm_bridge.waypoint_loader import WaypointLoader

DEFAULT_SPEED = 0.45  # m/s, matching pure_pursuit linear_speed


class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')

        self.declare_parameter('fleet_config_path', '')
        self.declare_parameter('waypoints_config_path', '')

        fleet_config_path = self.get_parameter('fleet_config_path').value
        waypoints_config_path = self.get_parameter('waypoints_config_path').value
        self.home_poses = self._load_home_poses(fleet_config_path)
        self.valid_robots = list(self.home_poses.keys())
        self.waypoint_loader = WaypointLoader(waypoints_config_path)

        self.fleet_state_sub = self.create_subscription(
            FleetState,
            '/fleet/robot_states',
            self.fleet_state_callback,
            10
        )

        self.goal_event_sub = self.create_subscription(
            GoalEvent,
            '/fleet/goal_events',
            self.goal_event_callback,
            10
        )

        self.decision_sub = self.create_subscription(
            String,
            '/llm/decision',
            self.decision_callback,
            10
        )

        self.fleet_navigate_client = ActionClient(self, FleetNavigate, '/fleet/fleet_navigate')
        self.fleet_set_state_client = self.create_client(FleetSetState, '/fleet/set_fleet_state')
        self.rotation_pubs = {}
        self.emergency_stop_pubs = {}

        self.robot_poses = {}
        for robot_id in self.valid_robots:
            odom_topic = f'/{robot_id}/platform/odom/filtered'
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10)
            self.get_logger().info(f'Subscribed to odometry for {robot_id} on {odom_topic}')

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

        self.latest_fleet_state = None
        self.get_logger().info('LLM bridge initialized')

    def _load_home_poses(self, path):
        if not path:
            return {}
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            home_poses = {}
            for entry in config.get('fleet_manager', {}).get('robots', []):
                ns = entry.get('namespace', '')
                home_pose = entry.get('home_pose', {})
                if ns and home_pose:
                    home_poses[ns] = {
                        'x': home_pose.get('x', 0.0),
                        'y': home_pose.get('y', 0.0),
                        'yaw': home_pose.get('yaw', 0.0)
                    }
            return home_poses
        except Exception as e:
            self.get_logger().error(f'Failed to load home poses from fleet config: {e}')
            return {}

    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg
        self.log_fleet_state(msg)

    def goal_event_callback(self, msg: GoalEvent):
        self.get_logger().info(f'Goal event: robot={msg.robot_id}, type={msg.type}')

    def odom_callback(self, msg: Odometry, robot_id: str):
        pose = msg.pose.pose
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.robot_poses[robot_id] = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw
        }

    def log_fleet_state(self, msg: FleetState):
        for i, robot_id in enumerate(msg.robot_ids):
            state = msg.states[i]
            status_parts = []
            if state.emergency_stop:
                status_parts.append('EMERGENCY_STOP')
            if state.waiting:
                status_parts.append('WAITING')
            if state.mission_active:
                status_parts.append('NAVIGATING')
            if state.idle:
                status_parts.append('IDLE')

            status = ', '.join(status_parts) if status_parts else 'UNKNOWN'
            self.get_logger().info(f'Robot {robot_id}: {status}')

    def decision_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse decision JSON: {e}')
            return

        missions = data.get('missions', [])
        self.get_logger().info(f'Received {len(missions)} mission(s) from LLM')

        navigate_missions = [m for m in missions if m.get('action') == 'navigate']
        set_state_missions = [m for m in missions if m.get('action') == 'set_state']
        rotate_missions = [m for m in missions if m.get('action') == 'rotate']
        emergency_stop_missions = [m for m in missions if m.get('action') == 'emergency_stop']
        go_home_missions = [m for m in missions if m.get('action') == 'go_home']
        clear_emergency_stop_missions = [m for m in missions if m.get('action') == 'clear_emergency_stop']

        if navigate_missions:
            self._send_navigate_missions(navigate_missions)

        if set_state_missions:
            self._send_set_state_missions(set_state_missions)

        if rotate_missions:
            self._send_rotate_missions(rotate_missions)

        if emergency_stop_missions:
            self._send_emergency_stop_missions(emergency_stop_missions)

        if go_home_missions:
            self._send_go_home_missions(go_home_missions)

        if clear_emergency_stop_missions:
            self._send_clear_emergency_stop_missions(clear_emergency_stop_missions)

    def _send_navigate_missions(self, missions):
        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Fleet navigate action server not available')
            return

        goal = FleetNavigate.Goal()
        for mission in missions:
            relative = mission.get('relative')
            if relative:
                resolved = self._resolve_relative(mission)
            else:
                resolved = self._resolve_waypoint(mission)

            if resolved is None:
                self.get_logger().error(f'Could not resolve waypoint for {mission["robot_id"]}')
                continue

            x, y, yaw = resolved
            fleet_goal = FleetGoal()
            fleet_goal.robot_id = mission['robot_id']

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            if yaw is not None:
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0

            fleet_goal.target_pose = pose
            goal.goals.append(fleet_goal)

        if goal.goals:
            self.get_logger().info(f'Sending {len(goal.goals)} navigate goal(s)')
            future = self.fleet_navigate_client.send_goal_async(goal)
            future.add_done_callback(self._navigate_goal_response_callback)

    def _resolve_waypoint(self, mission):
        waypoint_name = mission.get('waypoint_name')
        waypoint_names = mission.get('waypoint_names')
        waypoints = mission.get('waypoints')

        if waypoint_name:
            wp = self.waypoint_loader.get_waypoint(waypoint_name)
            if wp:
                return wp.get('x'), wp.get('y'), wp.get('yaw')
            self.get_logger().error(f'Unknown waypoint name: {waypoint_name}')
            return None

        if waypoint_names:
            wp = self.waypoint_loader.get_waypoint(waypoint_names[0])
            if wp:
                return wp.get('x'), wp.get('y'), wp.get('yaw')
            self.get_logger().error(f'Unknown waypoint name: {waypoint_names[0]}')
            return None

        if waypoints:
            wp = waypoints[0]
            if 'x' in wp and 'y' in wp:
                return wp['x'], wp['y'], wp.get('yaw')
            if 'lat' in wp and 'lon' in wp:
                x, y = self._gps_to_xy(wp['lat'], wp['lon'])
                return x, y, wp.get('yaw')

        return None

    def _resolve_relative(self, mission):
        robot_id = mission['robot_id']
        pose = self.robot_poses.get(robot_id)
        if pose:
            cx = pose['x']
            cy = pose['y']
            cyaw = pose['yaw']
        else:
            home = self.home_poses.get(robot_id)
            if home:
                self.get_logger().warn(f'No odometry for {robot_id}, using home pose as fallback for relative move')
                cx = home['x']
                cy = home['y']
                cyaw = home['yaw']
            else:
                self.get_logger().error(f'No odometry or home pose for {robot_id}, cannot resolve relative move')
                return None

        relative = mission.get('relative', {})
        forward = relative.get('forward', 0.0)
        right = relative.get('right', 0.0)

        x = cx + forward * math.cos(cyaw) + right * math.cos(cyaw - math.pi / 2.0)
        y = cy + forward * math.sin(cyaw) + right * math.sin(cyaw - math.pi / 2.0)

        return x, y, mission.get('relative_yaw')

    def _gps_to_xy(self, lat_deg, lon_deg):
        origin = self.waypoint_loader.gps_origin
        if not origin:
            self.get_logger().error('No GPS origin configured for coordinate conversion')
            return None, None

        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        x = (lon - origin['lon']) * math.cos(origin['lat']) * 6371000.0
        y = (lat - origin['lat']) * 6371000.0
        return x, y

    def _navigate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Fleet navigate goal rejected')
            return

        self.get_logger().info('Fleet navigate goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigate_result_callback)

    def _navigate_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Fleet navigation complete: {result.result}')

    def _send_set_state_missions(self, missions):
        if not self.fleet_set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Fleet set state service not available')
            return

        robot_ids = [m['robot_id'] for m in missions]
        waiting = any(m.get('waiting', False) for m in missions)
        avoidance_enabled = any(m.get('avoidance_enabled', False) for m in missions)

        request = FleetSetState.Request()
        request.robot_ids = robot_ids
        request.waiting = waiting
        request.avoidance_enabled = avoidance_enabled

        self.get_logger().info(f'Setting state for {robot_ids}: waiting={waiting}, avoidance={avoidance_enabled}')
        future = self.fleet_set_state_client.call_async(request)
        future.add_done_callback(self._set_state_response_callback)

    def _set_state_response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Fleet state set: {response.results}')

    def _send_rotate_missions(self, missions):
        for mission in missions:
            robot_id = mission.get('robot_id')
            angle_deg = mission.get('angle_deg', 0.0)
            
            # Create publisher for this robot if it doesn't exist
            if robot_id not in self.rotation_pubs:
                topic = f'/{robot_id}/rotation_goal'
                self.rotation_pubs[robot_id] = self.create_publisher(Float64, topic, 10)
            
            msg = Float64()
            msg.data = float(angle_deg)
            self.rotation_pubs[robot_id].publish(msg)
            self.get_logger().info(f'Published rotation goal to {robot_id}: {angle_deg} degrees')

    def _send_emergency_stop_missions(self, missions):
        for mission in missions:
            robot_id = mission.get('robot_id')
            
            if robot_id not in self.emergency_stop_pubs:
                topic = f'/{robot_id}/emergency_stop'
                self.emergency_stop_pubs[robot_id] = self.create_publisher(Bool, topic, 10)
            
            msg = Bool()
            msg.data = True
            self.emergency_stop_pubs[robot_id].publish(msg)
            self.get_logger().info(f'Emergency stop triggered for {robot_id}')

    def _send_go_home_missions(self, missions):
        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Fleet navigate action server not available')
            return

        goal = FleetNavigate.Goal()
        for mission in missions:
            robot_id = mission.get('robot_id')
            
            if robot_id not in self.home_poses:
                self.get_logger().error(f'No home pose configured for {robot_id}')
                continue
            
            home = self.home_poses[robot_id]
            fleet_goal = FleetGoal()
            fleet_goal.robot_id = robot_id

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(home['x'])
            pose.pose.position.y = float(home['y'])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            fleet_goal.target_pose = pose
            goal.goals.append(fleet_goal)

        if goal.goals:
            self.get_logger().info(f'Sending {len(goal.goals)} go_home goal(s)')
            future = self.fleet_navigate_client.send_goal_async(goal)
            future.add_done_callback(self._navigate_goal_response_callback)

    def _send_clear_emergency_stop_missions(self, missions):
        for mission in missions:
            robot_id = mission.get('robot_id')
            
            if robot_id not in self.emergency_stop_pubs:
                topic = f'/{robot_id}/emergency_stop'
                self.emergency_stop_pubs[robot_id] = self.create_publisher(Bool, topic, 10)
            
            msg = Bool()
            msg.data = False
            self.emergency_stop_pubs[robot_id].publish(msg)
            self.get_logger().info(f'Emergency stop cleared for {robot_id}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
