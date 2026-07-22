#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import sys


class SquarePatrol(Node):
    def __init__(self):
        super().__init__('square_patrol')
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_waypoints', 10)
        self.rotation_pub = self.create_publisher(Float64, 'rotation_goal', 10)
        self.odom_sub = self.create_subscription(Odometry, 'platform/odom', self.odom_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.waypoints = []
        self.rotations = []
        self.step_index = 0
        self.state = 'init'

        self.timer = self.create_timer(0.1, self.tick)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        if not self.odom_received:
            self.get_logger().info(f'Start pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')
        self.odom_received = True

    def build_patrol(self):
        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw

        side = 4.0
        rot_ccw = math.pi / 2.0

        cs = math.cos(self.start_yaw)
        sn = math.sin(self.start_yaw)

        p1_x = self.start_x + side * cs
        p1_y = self.start_y + side * sn

        p2_x = self.start_x + side * cs - side * sn
        p2_y = self.start_y + side * sn + side * cs

        p3_x = self.start_x - side * sn
        p3_y = self.start_y + side * cs

        self.waypoints = [
            (self.start_x, self.start_y),
            (p1_x, p1_y),
            (p2_x, p2_y),
            (p3_x, p3_y),
        ]
        self.rotations = [rot_ccw, rot_ccw, rot_ccw, rot_ccw]

    def tick(self):
        if not self.odom_received:
            return

        if self.state == 'init':
            self.get_logger().info('Building patrol route...')
            self.build_patrol()
            self.state = 'next'
            return

        if self.state == 'next':
            if self.step_index >= len(self.waypoints):
                self.state = 'done'
                return

            tx, ty = self.waypoints[self.step_index]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.3 and self.step_index == 0:
                self.step_index += 1
                return
            elif dist < 0.3:
                self.start_rotation()
                return
            else:
                self.start_drive(tx, ty)
                return

        if self.state == 'driving':
            tx, ty = self.waypoints[self.step_index]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.3:
                self.get_logger().info(f'  Arrived at ({tx:.2f}, {ty:.2f})')
                self.state = 'next'
            return

        if self.state == 'rotating':
            elapsed = (self.get_clock().now() - self.rotation_start_time).nanoseconds / 1e9
            yaw_delta = self.normalize_angle(self.yaw - self.rotation_start_yaw)
            remaining = abs(self.normalize_angle(self.rotation_target - yaw_delta))
            if remaining < 0.1 or elapsed > 12.0:
                self.get_logger().info(f'  Rotation complete (remaining={remaining:.3f} rad, elapsed={elapsed:.1f}s)')
                self.step_index += 1
                self.state = 'next'
            return

        if self.state == 'done':
            self.get_logger().info('Square patrol complete!')
            self.timer.cancel()

    def start_drive(self, tx, ty):
        self.get_logger().info(f'Driving to ({tx:.2f}, {ty:.2f})')
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.state = 'driving'

    def start_rotation(self):
        if self.step_index >= len(self.rotations):
            self.state = 'done'
            return
        angle = self.rotations[self.step_index]
        self.get_logger().info(f'Rotating {math.degrees(angle):.0f}° CCW')
        self.rotation_start_yaw = self.yaw
        self.rotation_start_time = self.get_clock().now()
        self.rotation_target = angle
        msg = Float64()
        msg.data = angle
        self.rotation_pub.publish(msg)
        self.state = 'rotating'

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = SquarePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
