#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from pinkylib import Camera
import tf2_ros
from geometry_msgs.msg import PoseStamped
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose


def get_transform_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """
    rvec, tvec → 4×4 transform matrix (camera → marker)
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')

        # 1) MarkerArray 퍼블리시 (camera_link frame)
        self.publisher = self.create_publisher(
            MarkerArray, '/aruco_markers', 10
        )

        # 2) PoseStamped 퍼블리시 (map frame, latched)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco_marker_pose',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.bridge = CvBridge()

        # 3) 카메라 초기화 & 캘리브레이션 로드
        self.cam = Camera()
        self.cam.set_calibration("camera_calibration.npz")
        try:
            self.cam.start(width=640, height=480)
        except Exception as e:
            self.get_logger().error(f"Camera failed to start: {e}")
            rclpy.shutdown()
            return

        # 4) TF listener (map ↔ base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5) base_link → camera_link 고정 변환 (카메라 높이 6cm)
        self.T_base2cam = np.eye(4, dtype=float)
        self.T_base2cam[2, 3] = 0.06

        # 6) 주기적 프레임 처리
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        try:
            frame = self.cam.get_frame()
            if frame is None:
                return

            # ArUco 검출 (marker_size=0.044m)
            result_frame, pose_list = self.cam.detect_aruco(
                frame,
                aruco_dict_type=cv2.aruco.DICT_6X6_250,
                marker_size=0.044
            )

            marker_array = MarkerArray()

            if not pose_list:
                return

            for i, pose in enumerate(pose_list):
                # 기본 Marker 설정
                marker = Marker()
                marker.header.frame_id = 'camera_link'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = 0.044
                marker.scale.y = 0.044
                marker.scale.z = 0.001
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # 경우 1: pose = [id, x, y, z]
                if isinstance(pose, (list, tuple)) and len(pose) == 4:
                    marker_id, x, y, z = pose
                    marker.id = int(marker_id)
                    marker.pose.position.x = float(x)
                    marker.pose.position.y = float(y)
                    marker.pose.position.z = float(z)
                    # 방향 정보 없으면 identity quaternion
                    marker.pose.orientation.w = 1.0

                    self.get_logger().info(
                        f"[MarkerArray] id={marker.id} at camera_link "
                        f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
                    )

                # 경우 2: pose = (rvec, tvec)
                elif isinstance(pose, (list, tuple)) and len(pose) == 2:
                    rvec, tvec = pose
                    marker.id = i
                    # 카메라 프레임 위치
                    marker.pose.position.x = float(tvec[0])
                    marker.pose.position.y = float(tvec[1])
                    marker.pose.position.z = float(tvec[2])
                    marker.pose.orientation.w = 1.0

                    # → map 프레임으로 변환
                    T_cam2marker = get_transform_matrix(rvec, tvec)

                    try:
                        tf_msg = self.tf_buffer.lookup_transform(
                            'map', 'base_link', rclpy.time.Time()
                        )
                        # map → base_link
                        trans = tf_msg.transform
                        translation = [
                            trans.translation.x,
                            trans.translation.y,
                            trans.translation.z
                        ]
                        quaternion_tf = [
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z
                        ]

                        # 회전 매트릭스
                        R_map2base = quat2mat(quaternion_tf)
                        T_map2base = compose(translation, R_map2base, [1, 1, 1])

                        # 최종 map → marker
                        T_map2marker = T_map2base @ self.T_base2cam @ T_cam2marker
                        x_map, y_map, z_map = T_map2marker[:3, 3]

                        # 방향도 추출
                        R_map2marker = T_map2marker[:3, :3]
                        quat_map = mat2quat(R_map2marker)  # [w, x, y, z]

                        # PoseStamped 메시지 작성
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = 'map'
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.pose.position.x = float(x_map)
                        pose_msg.pose.position.y = float(y_map)
                        pose_msg.pose.position.z = float(z_map)
                        pose_msg.pose.orientation.w = float(quat_map[0])
                        pose_msg.pose.orientation.x = float(quat_map[1])
                        pose_msg.pose.orientation.y = float(quat_map[2])
                        pose_msg.pose.orientation.z = float(quat_map[3])

                        # 퍼블리시
                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info(
                            f"[PoseStamped] marker={marker.id} "
                            f"x={x_map:.3f}, y={y_map:.3f}, z={z_map:.3f}"
                        )

                    except Exception as e:
                        self.get_logger().warn(f"TF 변환 실패: {e}")

                else:
                    self.get_logger().warn(f"Unsupported pose format: {pose}")
                    continue

                marker_array.markers.append(marker)

            # MarkerArray 퍼블리시
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
