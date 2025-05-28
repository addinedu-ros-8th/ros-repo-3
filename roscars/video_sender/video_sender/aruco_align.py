import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from packaging import version
import time
import math

class ArucoAlign(Node):
    def __init__(self):
        super().__init__('aruco_align')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 카메라 보정 파라미터 로드
        calib_path = '/home/pinky/ros-repo-3/roscars/video_sender/config/camera_calibration.npz'
        data = np.load(calib_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs   = data['distortion_coefficients']

        # OpenCV 버전에 따른 디텍터 설정
        self.cv_version = version.parse(cv2.__version__)
        if self.cv_version >= version.parse('4.7.0'):
            self.dictionary     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.detectorParams = cv2.aruco.DetectorParameters()
            self.detector       = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)
        else:
            self.dictionary     = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.detectorParams = cv2.aruco.DetectorParameters_create()

        # 제어 파라미터
        self.marker_length    = 0.044    # 마커 한 변 길이 (m)
        self.desired_dist     = 0.15     # 목표 정지 거리 (m)
        self.dist_threshold   = 0.005    # 거리 오차 임계치 (m)
        self.angle_threshold  = 0.05     # 각도 오차 임계치 (rad)
        self.max_angular      = 0.5      # 각속도 한계 (rad/s)
        self.max_linear       = 0.2      # 선속도 한계 (m/s)
        self.Kc               = 0.1      # 선속도 보정 계수

        # PID 게인
        self.Kp_forward  = 0.5
        self.Ki_forward  = 0.1
        self.Kd_forward  = 0.05

        self.Kp_angle    = 1.0
        self.Ki_angle    = 0.01
        self.Kd_angle    = 0.1

        # PID 상태값 초기화
        self.integral_z      = 0.0
        self.prev_z_err      = 0.0
        self.integral_angle  = 0.0
        self.prev_angle_err  = 0.0
        self.last_time       = time.time()

    def process_frame(self, frame: np.ndarray):
        # BGR→그레이 변환
        if frame.ndim == 3 and frame.shape[2] == 4:
            bgr = frame[:, :, 1:4]
        else:
            bgr = frame
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        # 아루코 검출
        if self.cv_version >= version.parse('4.7.0'):
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detectorParams)

        # 시간 차 dt 계산
        now = time.time()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now

        twist = Twist()

        if ids is not None and len(ids) > 0:
            # 첫 번째 마커 포즈 계산
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[0:1], self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            x_m = tvecs[0][0][0]
            z_m = tvecs[0][0][2]

            # 각도 오류 (rad) 및 거리 오류 (m)
            angle_err = math.atan2(x_m, z_m)
            dist_err  = z_m - self.desired_dist

            # 각도 PID
            self.integral_angle += angle_err * dt
            derivative_angle   = (angle_err - self.prev_angle_err) / dt
            ang_cmd = (
                self.Kp_angle * angle_err
                + self.Ki_angle * self.integral_angle
                + self.Kd_angle * derivative_angle
            )
            ang_cmd = float(np.clip(ang_cmd, -self.max_angular, self.max_angular))
            twist.angular.z = ang_cmd
            self.prev_angle_err = angle_err

            # 거리 PID + 보정
            self.integral_z += dist_err * dt
            derivative_z   = (dist_err - self.prev_z_err) / dt
            lin_cmd = (
                self.Kp_forward * dist_err
                + self.Ki_forward * self.integral_z
                + self.Kd_forward * derivative_z
            ) + self.Kc * abs(ang_cmd)
            lin_cmd = float(np.clip(lin_cmd, -self.max_linear, self.max_linear))

            # 정지 조건: 마커와 로봇이 일직선 정렬(각도 오류 작음) 및 목표 거리(3cm) 도달 시
            if abs(angle_err) < self.angle_threshold and abs(z_m - self.desired_dist) < self.dist_threshold:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # PID 상태 초기화
                self.integral_z      = 0.0
                self.integral_angle  = 0.0
            else:
                twist.linear.x = lin_cmd
            self.prev_z_err = dist_err

        # 퍼블리시
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoAlign()
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        node.process_frame(frame)
        cv2.imshow('ArucoAlign', frame)
        if cv2.waitKey(1) == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
