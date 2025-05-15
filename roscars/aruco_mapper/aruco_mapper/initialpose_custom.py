#!/usr/bin/env python3

import csv
import time
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

        # CSV에서 마커 ID → 맵 절대 좌표 로딩
        self.marker_map = {}
        csv_path = '/home/pinky/ros-repo-3/roscars/aruco_mapper/aruco_marker_positions.csv'
        with open(csv_path, encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [fn.strip().lower() for fn in reader.fieldnames]
            for row in reader:
                mid = int(row['marker_id'])
                self.marker_map[mid] = np.array([
                    float(row['x']), float(row['y']), float(row['z'])
                ])

        # 카메라 초기화 및 캘리브레이션 적용
        self.bridge = CvBridge()
        self.cam = Camera()
        self.cam.set_calibration(
            '/home/pinky/ros-repo-3/roscars/aruco_mapper/camera_calibration.npz'
        )
        try:
            self.cam.start(width=640, height=480)
        except Exception as e:
            self.get_logger().error(f'Camera failed to start: {e}')
            raise

        # ArUco 딕셔너리 및 파라미터
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 퍼블리셔 및 상태 플래그
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose_custom',
            10
        )
        self.published = False

    def capture_and_publish(self):
        frame = self.cam.get_frame()
        if frame is None:
            self.get_logger().error('No frame received')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )
        if ids is None or len(ids) == 0:
            self.get_logger().warn('No ArUco marker detected, retrying...')
            return

        marker_id = int(ids[0][0])
        if marker_id not in self.marker_map:
            self.get_logger().warn(f'Marker {marker_id} not in map, retrying...')
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            0.10,
            self.cam.calibration_matrix,
            self.cam.dist_coeffs
        )
        rvec, tvec = rvecs[0][0], tvecs[0][0]

        R_cam_marker, _ = cv2.Rodrigues(rvec)
        R_marker_cam = R_cam_marker.T
        t_marker_cam = -R_marker_cam.dot(tvec)
        marker_pos_map = self.marker_map[marker_id]
        cam_pos_map = marker_pos_map + t_marker_cam
        base_pos_map = cam_pos_map + np.array([0.0, 0.0, -0.06])

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
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    # 마커 인식 후 퍼블리시 될 때까지 반복
    while rclpy.ok() and not node.published:
        node.capture_and_publish()
        rclpy.spin_once(node, timeout_sec=0.5)
        time.sleep(0.5)

    node.get_logger().info('Initialpose published, shutting down')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()