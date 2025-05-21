import json
import time
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from transforms3d.quaternions import mat2quat
from picamera2 import Picamera2

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
        self.marker_map = {int(item['id']): np.array([float(item['x']), float(item['y']), 0.0]) for item in data}

        # 캘리브레이션 로드
        calib = np.load(npz_path)
        files = calib.files
        self.calib_mat = calib.get('calibration_matrix', calib.get('camera_matrix', calib[files[0]]))
        self.dist_coeffs = calib.get('dist_coeffs', calib.get('distortion_coefficients', calib[files[1] if len(files) > 1 else files[0]]))

        # 퍼블리셔 설정
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10)
        )

        # ArUco 세팅
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Picamera2 초기화
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={'format': 'XRGB8888', 'size': (640, 480)})
        self.picam2.configure(config)

        self.published = False

    def run_detection(self):
        """
        카메라를 켜고 마커가 감지될 때까지 프레임을 처리합니다.
        감지되면 위치 퍼블리시 후 카메라를 종료합니다.
        """
        self.picam2.start()
        time.sleep(0.5)  # 워밍업

        while rclpy.ok() and not self.published:
            frame = self.picam2.capture_array()
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is None or len(ids) == 0:
                # 우측 회전 명령
                twist = Twist()
                twist.angular.z = -1.0
                self.cmd_pub.publish(twist)
                continue

            # 마커 발견 시 회전 멈춤
            self.cmd_pub.publish(Twist())
            mid = int(ids[0][0])
            if mid not in self.marker_map:
                self.get_logger().warn(f'Marker {mid} not in map')
                continue

            # 자세 계산
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[0]], 0.044, self.calib_mat, self.dist_coeffs
            )
            rvec, tvec = rvecs[0][0], tvecs[0][0]
            mpos = self.marker_map[mid]

            # 오프셋 계산 (기존 로직)
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

            # 초기 위치 퍼블리시
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = float(x)
            pose_msg.pose.pose.position.y = float(y)
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation.w = quat[0]
            pose_msg.pose.pose.orientation.x = quat[1]
            pose_msg.pose.pose.orientation.y = quat[2]
            pose_msg.pose.pose.orientation.z = quat[3]
            pose_msg.pose.covariance = [0.05]*36

            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"✅ initialpose from marker {mid}: x={x:.3f}, y={y:.3f}")
            self.published = True

        # 마커 인식 후 카메라 종료
        self.picam2.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    node.run_detection()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()