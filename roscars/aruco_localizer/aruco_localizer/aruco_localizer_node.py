import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from transforms3d.quaternions import mat2quat


class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 캘리브레이션 불러오기 (안전한 경로 처리)
        calib_path = os.path.expanduser('~/ros-repo-3/roscars/aruco_localizer/camera_calibration.npz')
        with np.load(calib_path) as data:
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['distortion_coefficients']

        self.marker_size = 0.02  # 2cm

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corner, self.marker_size, self.camera_matrix, self.dist_coeffs
                )

                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'world'

                pose.pose.position.x = tvec[0][0][0]
                pose.pose.position.y = tvec[0][0][1]
                pose.pose.position.z = tvec[0][0][2]

                rot_matrix, _ = cv2.Rodrigues(rvec[0])
                quat = mat2quat(rot_matrix)  # [w, x, y, z]

                # ROS의 quaternion은 [x, y, z, w] 순서
                pose.pose.orientation.x = quat[1]
                pose.pose.orientation.y = quat[2]
                pose.pose.orientation.z = quat[3]
                pose.pose.orientation.w = quat[0]

                self.publisher.publish(pose)
                self.get_logger().info(f"Published ArUco Pose: {pose.pose.position}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
