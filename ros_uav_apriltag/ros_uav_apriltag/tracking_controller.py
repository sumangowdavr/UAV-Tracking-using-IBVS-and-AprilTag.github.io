#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

from ros_uav_apriltag.control_utils import compute_xy_velocity


class TrackingController(Node):
    def __init__(self):
        super().__init__('tracking_controller')
        self.declare_parameter('kp_xy', 0.6)
        self.declare_parameter('max_xy_vel_mps', 0.8)
        self.declare_parameter('center_tolerance_m', 0.2)
        self.declare_parameter('descent_rate_mps', 0.0)

        self.kp_xy = float(self.get_parameter('kp_xy').value)
        self.max_xy_vel = float(self.get_parameter('max_xy_vel_mps').value)
        self.center_tol = float(self.get_parameter('center_tolerance_m').value)
        self.descent_rate = float(self.get_parameter('descent_rate_mps').value)

        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_found = False

        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/mission/target_pose', self._target_pose_cb, 10)
        self.create_subscription(Bool, '/mission/target_found', self._target_found_cb, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/mission/tracking_cmd_vel', 10)
        self.centered_pub = self.create_publisher(Bool, '/mission/tracking_centered', 10)
        self.timer = self.create_timer(0.05, self._loop)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _target_pose_cb(self, msg):
        self.target_pose = msg

    def _target_found_cb(self, msg):
        self.target_found = bool(msg.data)

    def _loop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'

        if self.target_found:
            ex = self.target_pose.pose.position.x - self.current_pose.pose.position.x
            ey = self.target_pose.pose.position.y - self.current_pose.pose.position.y
            vx, vy = compute_xy_velocity(ex, ey, self.kp_xy, self.max_xy_vel)
            cmd.twist.linear.x = vx
            cmd.twist.linear.y = vy
            cmd.twist.linear.z = -abs(self.descent_rate)
            centered = math.sqrt(ex * ex + ey * ey) < self.center_tol
            self.centered_pub.publish(Bool(data=centered))
        else:
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = 0.0
            self.centered_pub.publish(Bool(data=False))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingController()
    node.get_logger().info('tracking_controller started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
