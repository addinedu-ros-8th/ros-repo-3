#!/usr/bin/env python3

import json
import numpy as np
import rclpy
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.quaternions import mat2quat
from transforms3d.euler import mat2euler

class ArucoInitialPosePublisher(Node):
    EAST_MARKER_IDS   = {2,3,13,14,15,16,17,18,28,29,30,34}
    WEST_MARKER_IDS   = {0,5,6,7,8,20,22,23,24,32}
    SOUTH_MARKER_IDS  = {1,4,11,19,21,25,26,27,31,35}
    NORTH_MARKER_IDS  = {9,10,12,33}
    MARKER_IDS_36 = {36}
    MARKER_IDS_37 = {37}
    MARKER_IDS_38 = {38}
    MARKER_IDS_39 = {39}

    def __init__(self):
        super().__init__('aruco_initialpose_publisher')

        # 파라미터 선언
        self.declare_parameter('marker_map_path', '')
        self.declare_parameter('calibration_file', '')
        map_path = self.get_parameter('marker_map_path').get_parameter_value().string_value
        npz_path = self.get_parameter('calibration_file').get_parameter_value().string_value

        # 마커 맵 로드
        with open(map_path, encoding='utf-8') as f:
            data = json.load(f)
        self.marker_map = {
            int(item['id']): np.array([float(item['x']), float(item['y']), 0.0])
            for item in data
        }

        # 캘리브레이션 로드 (동적 키)
        calib = np.load(npz_path)
        files = calib.files
        if 'calibration_matrix' in files:
            self.calib_mat = calib['calibration_matrix']
        elif 'camera_matrix' in files:
            self.calib_mat = calib['camera_matrix']
        else:
            self.calib_mat = calib[files[0]]
            self.get_logger().warn(f"Using '{files[0]}' as calibration_matrix")
        if 'dist_coeffs' in files:
            self.dist_coeffs = calib['dist_coeffs']
        elif 'distortion_coefficients' in files:
            self.dist_coeffs = calib['distortion_coefficients']
        else:
            self.dist_coeffs = calib[files[1] if len(files)>1 else files[0]]
            self.get_logger().warn("Using second array as dist_coeffs")

        # 브리지 및 퍼블리셔
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # 구독자
        self.published = False
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # ArUco 세팅
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        if self.published:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids)==0:
            self.get_logger().warn('No ArUco marker detected')
            return

        mid = int(ids[0][0])
        if mid not in self.marker_map:
            self.get_logger().warn(f'Marker {mid} not in map')
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[0]], 0.044, self.calib_mat, self.dist_coeffs
        )
        rvec, tvec = rvecs[0][0], tvecs[0][0]
        mpos = self.marker_map[mid]

        # ===== 로그 추가: 맵 좌표와 카메라 프레임 위치 출력 =====
        self.get_logger().info(f"[MARKER_MAP] id={mid}, map_pos={mpos.tolist()}")
        self.get_logger().info(f"[CAMERA_LINK] id={mid}, tvec={tvec.tolist()}")
        # =======================================================

        # 오프셋 계산
        if mid in self.EAST_MARKER_IDS:
            x = mpos[0] + tvec[2] + 0.07
            y = mpos[1] + abs(tvec[0]) if tvec[0]>=0 else mpos[1] - abs(tvec[0])
        elif mid in self.SOUTH_MARKER_IDS:
            y = mpos[1] - abs(tvec[2]) - 0.07
            x = mpos[0] - abs(tvec[0]) if tvec[0]>=0 else mpos[0] + abs(tvec[0])
        elif mid in self.WEST_MARKER_IDS:
            x = mpos[0] - abs(tvec[2]) - 0.07
            y = mpos[1] + abs(tvec[0]) if tvec[0]>=0 else mpos[1] - abs(tvec[0])
        elif mid in self.NORTH_MARKER_IDS:
            y = mpos[1] + abs(tvec[2]) + 0.07
            x = mpos[0] + abs(tvec[0]) if tvec[0]>=0 else mpos[0] - abs(tvec[0])
        elif mid in self.MARKER_IDS_36:
            y = mpos[1] + abs(tvec[2]/2) + 0.04
            x = mpos[0] + abs(tvec[2]/2) + 0.04
        elif mid in self.MARKER_IDS_37:
            y = mpos[1] - abs(tvec[2]/2) - 0.04
            x = mpos[0] + abs(tvec[2]/2) + 0.04
        elif mid in self.MARKER_IDS_38:
            y = mpos[1] - abs(tvec[2]/2) - 0.04
            x = mpos[0] - abs(tvec[2]/2) - 0.04
        elif mid in self.MARKER_IDS_39:
            y = mpos[1] + abs(tvec[2]/2) + 0.04
            x = mpos[0] - abs(tvec[2]/2) - 0.04
        else:
            x, y = mpos[0], mpos[1]

        # 회전 → 쿼터니언
        R_cm, _ = cv2.Rodrigues(rvec)
        quat = mat2quat(R_cm.T)

        # 퍼블리시
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]
        msg.pose.covariance = [0.05]*36

        self.pub.publish(msg)
        self.get_logger().info(f"✅ initialpose from marker {mid}: x={x:.3f}, y={y:.3f}")
        self.published = True

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()