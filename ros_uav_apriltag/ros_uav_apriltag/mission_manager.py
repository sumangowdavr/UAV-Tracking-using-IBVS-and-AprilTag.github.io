#!/usr/bin/env python3
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool


class MissionPhase(Enum):
    WAIT_FCU = 0
    TAKEOFF = 1
    SEARCH = 2
    TRACK = 3
    HOVER = 4


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('takeoff_altitude_m', 3.0)
        self.declare_parameter('hover_altitude_m', 2.5)
        self.declare_parameter('arm_timeout_s', 12.0)
        self.declare_parameter('center_hold_time_s', 2.0)

        self.rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.takeoff_alt = float(self.get_parameter('takeoff_altitude_m').value)
        self.hover_alt = float(self.get_parameter('hover_altitude_m').value)
        self.arm_timeout = float(self.get_parameter('arm_timeout_s').value)
        self.center_hold = float(self.get_parameter('center_hold_time_s').value)

        self.current_state = State()
        self.local_pose = PoseStamped()
        self.search_setpoint = PoseStamped()
        self.target_pose = PoseStamped()
        self.track_cmd = TwistStamped()

        self.target_found = False
        self.tracking_centered = False
        self.center_start_ns = None

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/mission/search_setpoint', self._search_sp_cb, 10)
        self.create_subscription(PoseStamped, '/mission/target_pose', self._target_pose_cb, 10)
        self.create_subscription(TwistStamped, '/mission/tracking_cmd_vel', self._track_cmd_cb, 10)
        self.create_subscription(Bool, '/mission/target_found', self._target_found_cb, 10)
        self.create_subscription(Bool, '/mission/tracking_centered', self._tracking_centered_cb, 10)

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 20)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 20)
        self.phase_pub = self.create_publisher(Bool, '/mission/is_tracking_phase', 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.phase = MissionPhase.WAIT_FCU
        self.stream_count = 0
        self.arm_started_ns = None

        self.create_timer(1.0 / max(self.rate_hz, 2.0), self._loop)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.local_pose = msg

    def _search_sp_cb(self, msg):
        self.search_setpoint = msg

    def _target_pose_cb(self, msg):
        self.target_pose = msg

    def _track_cmd_cb(self, msg):
        self.track_cmd = msg

    def _target_found_cb(self, msg):
        self.target_found = bool(msg.data)

    def _tracking_centered_cb(self, msg):
        self.tracking_centered = bool(msg.data)

    def _pose_sp(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation.w = 1.0
        return sp

    def _request_offboard_arm(self):
        if self.mode_client.service_is_ready() and self.current_state.mode != 'OFFBOARD':
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.mode_client.call_async(req)
        if self.arm_client.service_is_ready() and not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)

    def _loop(self):
        now_ns = self.get_clock().now().nanoseconds
        takeoff_sp = self._pose_sp(0.0, 0.0, self.takeoff_alt)

        if self.phase == MissionPhase.WAIT_FCU:
            self.pos_pub.publish(takeoff_sp)
            if self.current_state.connected:
                self.phase = MissionPhase.TAKEOFF
                self.arm_started_ns = now_ns
                self.get_logger().info('FCU connected -> TAKEOFF')
            return

        if self.phase == MissionPhase.TAKEOFF:
            self.pos_pub.publish(takeoff_sp)
            self.stream_count += 1
            if self.stream_count > int(self.rate_hz * 2.0):
                self._request_offboard_arm()
            if self.current_state.mode == 'OFFBOARD' and self.current_state.armed:
                self.phase = MissionPhase.SEARCH
                self.get_logger().info('Armed OFFBOARD -> SEARCH')
                return
            if self.arm_started_ns and (now_ns - self.arm_started_ns) / 1e9 > self.arm_timeout:
                self.get_logger().error('Arm/OFFBOARD timeout')
                rclpy.shutdown()
            return

        if self.phase == MissionPhase.SEARCH:
            self.phase_pub.publish(Bool(data=False))
            sp = self.search_setpoint
            if sp.header.frame_id == '':
                sp = takeoff_sp
            self.pos_pub.publish(sp)
            if self.target_found:
                self.phase = MissionPhase.TRACK
                self.center_start_ns = None
                self.get_logger().info('Tag detected -> TRACK')
            return

        if self.phase == MissionPhase.TRACK:
            self.phase_pub.publish(Bool(data=True))
            cmd = self.track_cmd
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(cmd)
            if self.tracking_centered:
                if self.center_start_ns is None:
                    self.center_start_ns = now_ns
                hold = (now_ns - self.center_start_ns) / 1e9
                if hold >= self.center_hold:
                    self.phase = MissionPhase.HOVER
                    self.get_logger().info('Centered -> HOVER')
            else:
                self.center_start_ns = None
            return

        if self.phase == MissionPhase.HOVER:
            self.phase_pub.publish(Bool(data=True))
            sp = self._pose_sp(
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.hover_alt,
            )
            self.pos_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    node.get_logger().info('mission_manager started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
