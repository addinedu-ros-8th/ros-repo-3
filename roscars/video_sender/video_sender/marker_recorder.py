import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import tf2_geometry_msgs
import csv
import os

class MarkerRecorder(Node):
    def __init__(self):
        super().__init__('marker_recorder')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/aruco_markers',
            self.marker_callback,
            10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.saved_ids = set()

        output_dir = os.path.expanduser('~/ros-repo-3/roscars/video_sender')
        os.makedirs(output_dir, exist_ok=True)
        self.output_file = os.path.join(output_dir, 'aruco_marker_positions.csv')

        with open(self.output_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['marker_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def marker_callback(self, msg):
        for marker in msg.markers:
            marker_id = marker.id
            if marker_id in self.saved_ids:
                continue

            try:
                stamped_pose = PoseStamped()
                stamped_pose.header = marker.header
                stamped_pose.pose = marker.pose

                transformed = self.tf_buffer.transform(
                    stamped_pose, 'map',
                    timeout=Duration(seconds=1.0)
                )

                self.get_logger().info(
                    f"[Saved] Marker {marker_id} → x: {transformed.pose.position.x:.2f}, y: {transformed.pose.position.y:.2f}"
                )

                with open(self.output_file, 'a') as f:
                    writer = csv.writer(f)
                    p = transformed.pose.position
                    o = transformed.pose.orientation
                    writer.writerow([marker_id, p.x, p.y, p.z, o.x, o.y, o.z, o.w])

                self.saved_ids.add(marker_id)

            except Exception as e:
                self.get_logger().warn(f"TF 변환 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
