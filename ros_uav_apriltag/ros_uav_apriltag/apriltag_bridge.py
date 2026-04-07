#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros


class AprilTagBridge(Node):
    def __init__(self):
        super().__init__('apriltag_bridge')

        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detections_topic', '/tag_detections')
        self.declare_parameter('target_pose_topic', '/mission/target_pose')
        self.declare_parameter('target_found_topic', '/mission/target_found')

        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        target_found_topic = str(self.get_parameter('target_found_topic').value)

        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)
        self.found_pub = self.create_publisher(Bool, target_found_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(AprilTagDetectionArray, detections_topic, self._detections_cb, 10)
        self.found_pub.publish(Bool(data=False))

    def _detections_cb(self, msg):
        for det in msg.detections:
            if self.target_tag_id not in list(det.id):
                continue

            pose = PoseStamped()
            pose.header = det.pose.header
            pose.pose = det.pose.pose.pose
            out_pose = pose

            if pose.header.frame_id != self.map_frame:
                try:
                    out_pose = self.tf_buffer.transform(
                        pose,
                        self.map_frame,
                        timeout=Duration(seconds=0.1),
                    )
                except Exception:
                    self.get_logger().warn('TF transform to map unavailable, using raw pose frame.')

            self.pose_pub.publish(out_pose)
            self.found_pub.publish(Bool(data=True))
            return

        self.found_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagBridge()
    node.get_logger().info('apriltag_bridge started')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
