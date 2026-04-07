#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from ros_uav_apriltag.control_utils import generate_lawnmower


class SearchPlanner(Node):
    def __init__(self):
        super().__init__('search_planner')
        self.declare_parameter('bounds', [0.0, 6.0, 0.0, 6.0])
        self.declare_parameter('lane_spacing_m', 2.0)
        self.declare_parameter('search_altitude_m', 3.0)
        self.declare_parameter('arrival_radius_m', 0.4)

        bounds = self.get_parameter('bounds').value
        lane_spacing = float(self.get_parameter('lane_spacing_m').value)
        altitude = float(self.get_parameter('search_altitude_m').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius_m').value)

        self.waypoints = generate_lawnmower((bounds[0], bounds[1], bounds[2], bounds[3]), lane_spacing, altitude)
        self.wp_idx = 0
        self.current_pose = PoseStamped()

        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)
        self.wp_pub = self.create_publisher(PoseStamped, '/mission/search_setpoint', 10)
        self.idx_pub = self.create_publisher(Int32, '/mission/search_waypoint_idx', 10)
        self.timer = self.create_timer(0.05, self._loop)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _dist(self, sp):
        dx = self.current_pose.pose.position.x - sp.pose.position.x
        dy = self.current_pose.pose.position.y - sp.pose.position.y
        dz = self.current_pose.pose.position.z - sp.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _loop(self):
        wp = self.waypoints[self.wp_idx % len(self.waypoints)]
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = float(wp[0])
        sp.pose.position.y = float(wp[1])
        sp.pose.position.z = float(wp[2])
        sp.pose.orientation.w = 1.0
        self.wp_pub.publish(sp)
        self.idx_pub.publish(Int32(data=self.wp_idx))

        if self._dist(sp) < self.arrival_radius:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = SearchPlanner()
    node.get_logger().info('search_planner started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
