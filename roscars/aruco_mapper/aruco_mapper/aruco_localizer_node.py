#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose


def get_transform_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = tvec.reshape(3)
    return T

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')

        # 파라미터
        self.declare_parameter('calibration_file', '')
        npz_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        calib = np.load(npz_path)
        files = calib.files
        if 'calibration_matrix' in files:
            self.calib_mat = calib['calibration_matrix']
        else:
            self.calib_mat = calib[files[0]]
            self.get_logger().warn(f"Using '{files[0]}' as calibration_matrix")
        if 'dist_coeffs' in files:
            self.dist_coeffs = calib['dist_coeffs']
        elif len(files)>1:
            self_dist = files[1]
            self.dist_coeffs = calib[self_dist]
            self.get_logger().warn(f"Using '{self_dist}' as dist_coeffs")
        else:
            self.dist_coeffs = np.zeros((5,))
            self.get_logger().warn("No dist_coeffs found, using zeros.")

        # 퍼블리셔들
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_markers', 10)
        self.pose_pub   = self.create_publisher(
            PoseStamped, '/aruco_marker_pose',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.T_base2cam = np.eye(4)
        self.T_base2cam[2,3] = 0.06

        # 브리지, ArUco 설정
        self.bridge     = CvBridge()
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 구독자
        self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.044, self.calib_mat, self.dist_coeffs)
        marker_array = MarkerArray()

        for i, mid in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            # MarkerArray 제작
            marker = Marker()
            marker.header.frame_id = 'camera_link'
            marker.header.stamp    = msg.header.stamp
            marker.type   = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = 0.044
            marker.scale.z = 0.001
            marker.color.r = 1.0; marker.color.a = 1.0
            marker.id = int(mid)
            marker.pose.position.x = float(tvec[0])
            marker.pose.position.y = float(tvec[1])
            marker.pose.position.z = float(tvec[2])
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)

            # 카메라 프레임 위치 로그
            self.get_logger().info(f"[camera_link] marker={mid} x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}")

            # map 프레임 변환
            T_cam2marker = get_transform_matrix(rvecs[i][0], tvec)
            try:
                tf_msg = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                trans = tf_msg.transform
                quat_tf = [trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z]
                t_vec   = [trans.translation.x, trans.translation.y, trans.translation.z]
                R_map2base = quat2mat(quat_tf)
                T_map2base = compose(t_vec, R_map2base, [1,1,1])
                T_map2marker = T_map2base @ self.T_base2cam @ T_cam2marker
                pos = T_map2marker[:3,3]
                quat_map = mat2quat(T_map2marker[:3,:3])
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp    = msg.header.stamp
                pose_msg.pose.position.x = float(pos[0])
                pose_msg.pose.position.y = float(pos[1])
                pose_msg.pose.position.z = float(pos[2])
                pose_msg.pose.orientation.w = quat_map[0]
                pose_msg.pose.orientation.x = quat_map[1]
                pose_msg.pose.orientation.y = quat_map[2]
                pose_msg.pose.orientation.z = quat_map[3]
                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f"[map] marker={mid} x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}")
            except Exception as e:
                self.get_logger().warn(f"TF 변환 실패: {e}")

        self.marker_pub.publish(marker_array)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()