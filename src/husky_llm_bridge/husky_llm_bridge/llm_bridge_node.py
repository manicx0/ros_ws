#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
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

    def send_fleet_goals(self, goals_data):
        if not self.fleet_navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Fleet navigate action server not available')
            return None

        goal = FleetNavigate.Goal()
        for robot_id, pose_data in goals_data.items():
            fleet_goal = FleetGoal()
            fleet_goal.robot_id = robot_id

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = pose_data['x']
            pose.pose.position.y = pose_data['y']
            pose.pose.position.z = pose_data['z']
            pose.pose.orientation.w = 1.0
            fleet_goal.target_pose = pose

            goal.goals.append(fleet_goal)

        self.get_logger().info(f'Sending fleet goals: {goals_data}')
        future = self.fleet_navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Fleet goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f'Fleet navigation complete: {result.result}')
        return result.result

    def set_fleet_state(self, robot_ids, waiting, avoidance_enabled):
        if not self.fleet_set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Fleet set state service not available')
            return None

        request = FleetSetState.Request()
        request.robot_ids = robot_ids
        request.waiting = waiting
        request.avoidance_enabled = avoidance_enabled

        future = self.fleet_set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info(f'Fleet state set: {response.results}')
        return response.results


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
