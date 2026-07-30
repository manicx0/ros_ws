#!/usr/bin/env python3
import json
import math
import time
import yaml
import rclpy
from collections import deque
from dataclasses import dataclass, field
from typing import Optional
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Odometry
from husky_msgs.msg import FleetState, FleetGoal, GoalEvent
from husky_msgs.action import FleetNavigate
from husky_msgs.srv import FleetSetState
from husky_llm_bridge.waypoint_loader import WaypointLoader


@dataclass
class MissionEntry:
    command_id: str
    mission_idx: int
    action: str
    robot_id: str
    params: dict
    state: str = 'PENDING'
    reason: str = ''
    created_at: float = field(default_factory=time.time)
    started_at: float = 0.0
    completed_at: float = 0.0


class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')

        self.declare_parameter('fleet_config_path', '')
        self.declare_parameter('waypoints_config_path', '')
        self.declare_parameter('navigation_timeout', 60.0)

        fleet_config_path = self.get_parameter('fleet_config_path').value
        waypoints_config_path = self.get_parameter('waypoints_config_path').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
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

        self.decision_status_sub = self.create_subscription(
            String,
            '/llm/decision_status',
            self.decision_status_callback,
            10
        )

        self.command_status_sub = self.create_subscription(
            String,
            '/llm/command_status',
            self.command_status_callback,
            10
        )

        self.fleet_navigate_client = ActionClient(self, FleetNavigate, '/fleet/fleet_navigate')
        self.fleet_set_state_client = self.create_client(FleetSetState, '/fleet/set_fleet_state')
        self.rotation_pubs = {}
        self.emergency_stop_pubs = {}

        self.mission_status_pub = self.create_publisher(String, '/llm/mission_status', 10)
        self.mission_queue_pub = self.create_publisher(String, '/llm/mission_queue', 10)

        self._pending_queue = deque()
        self._active_mission: Optional[MissionEntry] = None
        self._history = []

        self.robot_poses = {}
        for robot_id in self.valid_robots:
            odom_topic = f'/{robot_id}/platform/odom'
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
        self.get_logger().info('LLM bridge initialized as mission lifecycle manager')
        
        self.timeout_timer = self.create_timer(5.0, self._check_timeouts)

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
        self.get_logger().info(f'Decision received: {msg.data[:100]}...')
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse decision JSON: {e}')
            return

        missions = data.get('missions', [])
        self.get_logger().info(f'Received {len(missions)} mission(s) from LLM')

        for idx, mission in enumerate(missions):
            action = mission.get('action', 'unknown')
            robot_id = mission.get('robot_id', 'unknown')
            params = {k: v for k, v in mission.items() if k not in ('action', 'robot_id')}

            entry = MissionEntry(
                command_id=data.get('command_id', 'unknown'),
                mission_idx=idx,
                action=action,
                robot_id=robot_id,
                params=params
            )
            self._pending_queue.append(entry)
            self.get_logger().info(f'Queued mission [{idx}]: {action} for {robot_id}')

        self._publish_queue_status()
        self._process_queue()

    def decision_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get('success') is False:
            reason = data.get('reason', 'Unknown planning error')
            self.get_logger().warn(f'Planning failed: {reason}')

            entry = MissionEntry(
                command_id='unknown',
                mission_idx=0,
                action='unknown',
                robot_id='unknown',
                params={},
                state='PLANNING_FAILED',
                reason=reason,
                completed_at=time.time()
            )
            self._history.append(entry)
            if len(self._history) > 10:
                self._history.pop(0)

            self._publish_mission_status(entry)
            self._publish_queue_status()

    def command_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        status = data.get('status', '')
        if status == 'failed':
            reason = data.get('reason', 'Unknown error')
            self.get_logger().warn(f'Command failed: {reason}')

    def _process_queue(self):
        if self._active_mission is not None:
            return

        while self._pending_queue:
            self._active_mission = self._pending_queue.popleft()
            if self._active_mission.state in ('PLANNING_FAILED',):
                self._archive_mission()
                self._active_mission = None
                continue
            break

        if self._active_mission is None:
            self._publish_queue_status()
            return

        self._active_mission.state = 'EXECUTING'
        self._active_mission.started_at = time.time()
        self._publish_mission_status(self._active_mission)

        action = self._active_mission.action
        if action == 'navigate':
            self._execute_navigate(self._active_mission)
        elif action == 'set_state':
            self._execute_set_state(self._active_mission)
        elif action == 'rotate':
            self._execute_rotate(self._active_mission)
        elif action == 'emergency_stop':
            self._execute_emergency_stop(self._active_mission)
        elif action == 'go_home':
            self._execute_go_home(self._active_mission)
        elif action == 'clear_emergency_stop':
            self._execute_clear_emergency_stop(self._active_mission)
        else:
            self.get_logger().error(f'Unknown action: {action}')
            self._mission_completed(False, f'Unknown action: {action}')

    def _mission_completed(self, success, reason):
        if self._active_mission is None:
            return
        self._active_mission.state = 'SUCCEEDED' if success else 'EXECUTED_FAILED'
        self._active_mission.reason = reason
        self._active_mission.completed_at = time.time()
        self._publish_mission_status(self._active_mission)
        self._archive_mission()
        self._active_mission = None
        self._publish_queue_status()
        self._process_queue()

    def _check_timeouts(self):
        if self._active_mission is None:
            return
        if self._active_mission.started_at == 0.0:
            return
        elapsed = time.time() - self._active_mission.started_at
        if elapsed > self.navigation_timeout:
            self.get_logger().warn(
                f'Mission {self._active_mission.action} for {self._active_mission.robot_id} '
                f'timed out after {elapsed:.1f}s (limit: {self.navigation_timeout}s)'
            )
            self._mission_completed(False, f'Navigation timeout after {elapsed:.1f}s')

    def _archive_mission(self):
        if self._active_mission is None:
            return
        self._history.append(self._active_mission)
        if len(self._history) > 10:
            self._history.pop(0)

    def _publish_mission_status(self, entry: MissionEntry):
        elapsed = (time.time() - entry.started_at) if entry.started_at else 0.0
        msg = String()
        msg.data = json.dumps({
            'command_id': entry.command_id,
            'mission_idx': entry.mission_idx,
            'action': entry.action,
            'robot_id': entry.robot_id,
            'state': entry.state,
            'reason': entry.reason,
            'elapsed': round(elapsed, 1)
        })
        self.mission_status_pub.publish(msg)

    def _publish_queue_status(self):
        active = None
        if self._active_mission:
            m = self._active_mission
            elapsed = (time.time() - m.started_at) if m.started_at else 0.0
            active = {
                'command_id': m.command_id,
                'mission_idx': m.mission_idx,
                'action': m.action,
                'robot_id': m.robot_id,
                'state': m.state,
                'elapsed': round(elapsed, 1)
            }
        pending = []
        for m in self._pending_queue:
            pending.append({
                'command_id': m.command_id,
                'mission_idx': m.mission_idx,
                'action': m.action,
                'robot_id': m.robot_id,
                'state': m.state
            })
        history = []
        for m in self._history:
            elapsed = (m.completed_at - m.created_at) if m.completed_at and m.created_at else 0.0
            history.append({
                'command_id': m.command_id,
                'mission_idx': m.mission_idx,
                'action': m.action,
                'robot_id': m.robot_id,
                'state': m.state,
                'reason': m.reason,
                'duration': round(elapsed, 1)
            })
        msg = String()
        msg.data = json.dumps({'active': active, 'pending': pending, 'history': history})
        self.mission_queue_pub.publish(msg)

    def _execute_navigate(self, mission: MissionEntry):
        robot_id = mission.robot_id
        params = mission.params

        relative = params.get('relative')
        if relative:
            resolved = self._resolve_relative(mission)
        else:
            resolved = self._resolve_waypoint(params)

        if resolved is None:
            self.get_logger().error(f'Could not resolve waypoint for {robot_id}')
            self._mission_completed(False, 'Could not resolve waypoint')
            return

        x, y, yaw = resolved
        self.get_logger().info(f'Resolved waypoint: x={x}, y={y}, yaw={yaw}')

        fleet_goal = FleetGoal()
        fleet_goal.robot_id = robot_id

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

        goal = FleetNavigate.Goal()
        goal.goals.append(fleet_goal)

        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self._mission_completed(False, 'Fleet navigate action server not available')
            return

        self.get_logger().info(f'Sending navigate goal for {robot_id}')
        future = self.fleet_navigate_client.send_goal_async(goal)
        future.add_done_callback(self._navigate_goal_response_callback)

    def _resolve_waypoint(self, params):
        waypoint_name = params.get('waypoint_name')
        waypoint_names = params.get('waypoint_names')
        waypoints = params.get('waypoints')

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

    def _resolve_relative(self, mission: MissionEntry):
        robot_id = mission.robot_id
        pose = self.robot_poses.get(robot_id)
        if pose:
            cx = pose['x']
            cy = pose['y']
            cyaw = pose['yaw']
            self.get_logger().info(f'Using odometry for {robot_id}: x={cx:.2f}, y={cy:.2f}, yaw={cyaw:.2f}')
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

        relative = mission.params.get('relative', {})
        forward = relative.get('forward', 0.0)
        right = relative.get('right', 0.0)

        x = cx + forward * math.cos(cyaw) + right * math.cos(cyaw - math.pi / 2.0)
        y = cy + forward * math.sin(cyaw) + right * math.sin(cyaw - math.pi / 2.0)
        
        self.get_logger().info(f'Resolved relative move: forward={forward}, right={right} → waypoint x={x:.2f}, y={y:.2f}')

        return x, y, mission.params.get('relative_yaw')

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
            self._mission_completed(False, 'Fleet navigate goal rejected')
            return

        self.get_logger().info('Fleet navigate goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigate_result_callback)

    def _navigate_result_callback(self, future):
        result = future.result()
        success = all(r.success for r in result.result.results)
        reason = 'Navigation complete' if success else (result.result.results[0].message if result.result.results else 'Unknown error')
        self._mission_completed(success, reason)

    def _execute_set_state(self, mission: MissionEntry):
        if not self.fleet_set_state_client.wait_for_service(timeout_sec=5.0):
            self._mission_completed(False, 'Fleet set state service not available')
            return

        robot_ids = [mission.robot_id]
        waiting = mission.params.get('waiting', False)
        avoidance_enabled = mission.params.get('avoidance_enabled', False)

        request = FleetSetState.Request()
        request.robot_ids = robot_ids
        request.waiting = waiting
        request.avoidance_enabled = avoidance_enabled

        self.get_logger().info(f'Setting state for {robot_ids}: waiting={waiting}, avoidance={avoidance_enabled}')
        future = self.fleet_set_state_client.call_async(request)
        future.add_done_callback(self._set_state_response_callback)

    def _set_state_response_callback(self, future):
        response = future.result()
        success = all(r.success for r in response.results)
        reason = 'State updated' if success else (response.results[0].message if response.results else 'Unknown error')
        self._mission_completed(success, reason)

    def _execute_rotate(self, mission: MissionEntry):
        robot_id = mission.robot_id
        angle_deg = mission.params.get('angle_deg', 0.0)
        
        if robot_id not in self.rotation_pubs:
            topic = f'/{robot_id}/rotation_goal'
            self.rotation_pubs[robot_id] = self.create_publisher(Float64, topic, 10)
        
        msg = Float64()
        msg.data = float(angle_deg)
        self.rotation_pubs[robot_id].publish(msg)
        self.get_logger().info(f'Published rotation goal to {robot_id}: {angle_deg} degrees')
        self._mission_completed(True, f'Rotation {angle_deg}° sent')

    def _execute_emergency_stop(self, mission: MissionEntry):
        robot_id = mission.robot_id
        
        if robot_id not in self.emergency_stop_pubs:
            topic = f'/{robot_id}/emergency_stop'
            self.emergency_stop_pubs[robot_id] = self.create_publisher(Bool, topic, 10)
        
        msg = Bool()
        msg.data = True
        self.emergency_stop_pubs[robot_id].publish(msg)
        self.get_logger().info(f'Emergency stop triggered for {robot_id}')
        self._mission_completed(True, 'Emergency stop sent')

    def _execute_go_home(self, mission: MissionEntry):
        robot_id = mission.robot_id
        
        if robot_id not in self.home_poses:
            self._mission_completed(False, f'No home pose configured for {robot_id}')
            return

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

        goal = FleetNavigate.Goal()
        goal.goals.append(fleet_goal)

        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self._mission_completed(False, 'Fleet navigate action server not available')
            return

        self.get_logger().info(f'Sending go_home goal for {robot_id}')
        future = self.fleet_navigate_client.send_goal_async(goal)
        future.add_done_callback(self._navigate_goal_response_callback)

    def _execute_clear_emergency_stop(self, mission: MissionEntry):
        robot_id = mission.robot_id
        
        if robot_id not in self.emergency_stop_pubs:
            topic = f'/{robot_id}/emergency_stop'
            self.emergency_stop_pubs[robot_id] = self.create_publisher(Bool, topic, 10)
        
        msg = Bool()
        msg.data = False
        self.emergency_stop_pubs[robot_id].publish(msg)
        self.get_logger().info(f'Emergency stop cleared for {robot_id}')
        self._mission_completed(True, 'Emergency stop cleared')


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
