import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from pinkylib import Camera
import tf2_ros
import tf_transformations
from geometry_msgs.msg import PoseStamped


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
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

        self.bridge = CvBridge()

        self.cam = Camera()
        self.cam.set_calibration("camera_calibration.npz")
        try:
            self.cam.start(width=640, height=480)
        except Exception as e:
            self.get_logger().error(f"Camera failed to start: {e}")
            rclpy.shutdown()
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.T_base2cam = np.eye(4)
        self.T_base2cam[2, 3] = 0.06

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

                    if isinstance(pose, (list, tuple)) and len(pose) == 4:
                        marker.id = int(pose[0])
                        x, y, z = float(pose[1]), float(pose[2]), float(pose[3])
                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        marker.pose.orientation.w = 1.0
                        self.get_logger().info(f"id: {pose[0]} x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")

                    elif isinstance(pose, (list, tuple)) and len(pose) == 2:
                        rvec, tvec = pose
                        marker.id = i
                        marker.pose.position.x = float(tvec[0])
                        marker.pose.position.y = float(tvec[1])
                        marker.pose.position.z = float(tvec[2])
                        marker.pose.orientation.w = 1.0

                        # map 좌표계로 변환
                        T_cam2marker = get_transform_matrix(rvec, tvec)
                        try:
                            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                            T_map2base = tf_transformations.concatenate_matrices(
                                tf_transformations.translation_matrix([
                                    tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z]),
                                tf_transformations.quaternion_matrix([
                                    tf.transform.rotation.x,
                                    tf.transform.rotation.y,
                                    tf.transform.rotation.z,
                                    tf.transform.rotation.w]))

                            T_map2marker = T_map2base @ self.T_base2cam @ T_cam2marker
                            x, y, z = T_map2marker[:3, 3]

                            pose_msg = PoseStamped()
                            pose_msg.header.frame_id = 'map'
                            pose_msg.header.stamp = self.get_clock().now().to_msg()
                            pose_msg.pose.position.x = x
                            pose_msg.pose.position.y = y
                            pose_msg.pose.position.z = z
                            pose_msg.pose.orientation.w = 1.0
                            self.pose_pub.publish(pose_msg)

                        except Exception as e:
                            self.get_logger().warn(f"TF 변환 실패: {e}")

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
