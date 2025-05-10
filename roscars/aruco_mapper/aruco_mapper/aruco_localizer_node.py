import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from pinkylib import Camera

def get_transform_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')

        self.publisher = self.create_publisher(MarkerArray, '/aruco_markers', 10)
        self.bridge = CvBridge()

        self.cam = Camera()
        self.cam.set_calibration("camera_calibration.npz")
        try:
            self.cam.start(width=640, height=480)
        except Exception as e:
            self.get_logger().error(f"Camera failed to start: {e}")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        try:
            frame = self.cam.get_frame()
            result_frame, pose_list = self.cam.detect_aruco(
                frame,
                aruco_dict_type=cv2.aruco.DICT_6X6_250,
                marker_size=0.02
            )

            marker_array = MarkerArray()

            if pose_list:
                for i, pose in enumerate(pose_list):
                    marker = Marker()
                    marker.header.frame_id = 'camera_link'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.scale.x = 0.02
                    marker.scale.y = 0.02
                    marker.scale.z = 0.001
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                    # ✅ 형식 1: [id, x, y, z]
                    if isinstance(pose, (list, tuple)) and len(pose) == 4:
                        marker.id = int(pose[0])
                        marker.pose.position.x = float(pose[1])
                        marker.pose.position.y = float(pose[2])
                        marker.pose.position.z = float(pose[3])
                        marker.pose.orientation.w = 1.0  # 기본 orientation

                        self.get_logger().info(
                            f"id: {pose[0]} x: {pose[1]:.2f}, y: {pose[2]:.2f}, z: {pose[3]:.2f}"
                        )

                    # ✅ 형식 2: (rvec, tvec)
                    elif isinstance(pose, (list, tuple)) and len(pose) == 2:
                        rvec, tvec = pose
                        marker.id = i
                        marker.pose.position.x = float(tvec[0])
                        marker.pose.position.y = float(tvec[1])
                        marker.pose.position.z = float(tvec[2])
                        marker.pose.orientation.w = 1.0  # 필요 시 변환 추가

                    else:
                        self.get_logger().warn(f"Unsupported pose format: {pose}")
                        continue

                    marker_array.markers.append(marker)

            self.publisher.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f"Aruco detection failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cam.close()
        node.destroy_node()
        rclpy.shutdown()
