#!/usr/bin/env python3
import os, csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2, cv2.aruco as aruco
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ArucoLogger(Node):
    def __init__(self):
        super().__init__('aruco_logger')
        # 파라미터 선언 및 읽기
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_origin', [0.0, 0.0])
        self.declare_parameter('output_csv', os.path.expanduser('~/aruco_log.csv'))

        img_topic   = self.get_parameter('image_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        origin = self.get_parameter('map_origin').value
        self.origin_x, self.origin_y = origin
        csv_path = self.get_parameter('output_csv').value

        # CSV 준비
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['stamp','id','x_map','y_map','z_map','qx','qy','qz','qw'])

        # OpenCV ArUco 준비
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        # TF 준비
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 동기화 구독 (이미지 & 카메라 정보)
        sub_img  = Subscriber(self, Image,      img_topic)
        sub_info = Subscriber(self, CameraInfo, info_topic)
        sync = ApproximateTimeSynchronizer([sub_img, sub_info], queue_size=10, slop=0.1)
        sync.registerCallback(self.cb)

        self.get_logger().info(f'ArucoLogger started, writing to {csv_path}')

    def cb(self, img_msg, info_msg):
        # 1) 이미지 → OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # 2) 마커 검출
        corners, ids, _ = aruco.detectMarkers(cv_img, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            return

        # 3) 카메라 파라미터로 pose 추정
        K = [[info_msg.k[i] for i in range(3)] for _ in range(3)]
        dist = list(info_msg.d)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, K, dist)

        for idx, marker_id in enumerate(ids.flatten()):
            # 카메라 프레임 기준 PoseStamped
            ps_cam = PoseStamped()
            ps_cam.header = img_msg.header
            ps_cam.pose.position.x = float(tvecs[idx][0][0])
            ps_cam.pose.position.y = float(tvecs[idx][0][1])
            ps_cam.pose.position.z = float(tvecs[idx][0][2])

            # 회전벡터→쿼터니언
            rotM, _ = cv2.Rodrigues(rvecs[idx][0])
            # (여기선 간략히 rotation 없이 로그할 수도 있고, Quaternion 계산 라이브러리 사용)

            # 4) map_frame 으로 변환 (tf2)
            try:
                ps_map = self.tf_buffer.transform(ps_cam, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f'TF transform failed: {e}')
                continue

            # 5) map_origin 적용
            p = ps_map.pose.position
            p.x += self.origin_x
            p.y += self.origin_y

            # 6) CSV 기록
            q = ps_map.pose.orientation
            self.csv_writer.writerow([
                ps_map.header.stamp.sec + ps_map.header.stamp.nanosec*1e-9,
                marker_id,
                f'{p.x:.3f}', f'{p.y:.3f}', f'{p.z:.3f}',
                f'{q.x:.3f}', f'{q.y:.3f}', f'{q.z:.3f}', f'{q.w:.3f}',
            ])
            self.get_logger().info(f'Marker {marker_id} @ map({p.x:.2f},{p.y:.2f})')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = ArucoLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
