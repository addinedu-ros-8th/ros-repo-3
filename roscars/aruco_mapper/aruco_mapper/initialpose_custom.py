#!/usr/bin/env python3
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from pinkylib import Camera
import cv2

class ArucoInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_initialpose_publisher')

        # 1) CSV에서 마커 ID → 맵 절대 좌표 로딩
        self.marker_map = {}  # marker_id → [x, y, z]
        csv_path = '/home/pinky/ros-repo-3/roscars/aruco_mapper/aruco_marker_positions.csv'
        with open(csv_path, encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            # 필드명 보정: 공백 제거·소문자 통일
            reader.fieldnames = [fn.strip().lower() for fn in reader.fieldnames]
            for row in reader:
                mid = int(row['marker_id'])
                self.marker_map[mid] = np.array([
                    float(row['x']), float(row['y']), float(row['z'])
                ])

        # 2) 카메라 초기화 및 캘리브레이션 파일 적용
        self.bridge = CvBridge()
        self.cam = Camera()
        self.cam.set_calibration('camera_calibration.npz')
        try:
            self.cam.start(width=640, height=480)
        except Exception as e:
            self.get_logger().error(f'Camera failed to start: {e}')
            rclpy.shutdown()
            return

        # 3) ArUco 딕셔너리 및 파라미터 설정
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 4) /initialpose 퍼블리셔
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose_custom', 10)

        # 5) 타이머(초당 2회)로 검출 반복
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        frame = self.cam.get_frame()
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 5.1) 마커 검출
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        if ids is None:
            return

        # 5.2) 첫 번째 마커만 사용
        marker_id = int(ids[0][0])
        if marker_id not in self.marker_map:
            return

        # 5.3) 포즈 추정 (카메라 → 마커 변환)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 0.10,
            self.cam.calibration_matrix,
            self.cam.dist_coeffs
        )
        rvec, tvec = rvecs[0][0], tvecs[0][0]

        # 5.4) 카메라의 맵 좌표 계산
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        R_marker_cam   = R_cam_marker.T
        t_marker_cam   = -R_marker_cam.dot(tvec)
        marker_pos_map = self.marker_map[marker_id]
        cam_pos_map    = marker_pos_map + t_marker_cam

        # 5.5) 지면으로부터 카메라 높이 보정
        base_pos_map = cam_pos_map + np.array([0.0, 0.0, -0.06])

        # 6) 메시지 구성 및 발행
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(base_pos_map[0])
        msg.pose.pose.position.y = float(base_pos_map[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.05] * 36

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published initialpose from marker {marker_id}: '
            f'x={base_pos_map[0]:.2f}, y={base_pos_map[1]:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
