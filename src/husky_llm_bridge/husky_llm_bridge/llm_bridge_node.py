#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from husky_msgs.msg import FleetState, FleetGoal, GoalEvent
from husky_msgs.action import FleetNavigate
from husky_msgs.srv import FleetSetState


class LLMBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_bridge_node')

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

        self.latest_fleet_state = None
        self.get_logger().info('LLM bridge initialized')

    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg
        self.log_fleet_state(msg)

    def goal_event_callback(self, msg: GoalEvent):
        self.get_logger().info(f'Goal event: robot={msg.robot_id}, type={msg.type}')

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

        if navigate_missions:
            self._send_navigate_missions(navigate_missions)

        if set_state_missions:
            self._send_set_state_missions(set_state_missions)

    def _send_navigate_missions(self, missions):
        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Fleet navigate action server not available')
            return

        goal = FleetNavigate.Goal()
        for mission in missions:
            fleet_goal = FleetGoal()
            fleet_goal.robot_id = mission['robot_id']

            pose = PoseStamped()
            pose.header.frame_id = 'odom'

            waypoints = mission.get('waypoints', [])
            if waypoints:
                wp = waypoints[0]
                pose.pose.position.x = float(wp['x'])
                pose.pose.position.y = float(wp['y'])
                pose.pose.position.z = float(wp.get('z', 0.0))
                pose.pose.orientation.w = 1.0

            fleet_goal.target_pose = pose
            goal.goals.append(fleet_goal)

        self.get_logger().info(f'Sending {len(goal.goals)} navigate goal(s)')
        future = self.fleet_navigate_client.send_goal_async(goal)
        future.add_done_callback(self._navigate_goal_response_callback)

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
