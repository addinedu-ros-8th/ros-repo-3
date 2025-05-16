#!/usr/bin/env python3

import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from pinkylib import Camera
import cv2
from transforms3d.quaternions import mat2quat
from transforms3d.euler import mat2euler  # 추가

class ArucoInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_initialpose_publisher')

        # Load marker map from JSON
        self.marker_map = {}
        json_path = '/home/pinky/ros-repo-3/roscars/aruco_mapper/aruco_marker_positions.json'
        with open(json_path, encoding='utf-8') as f:
            data = json.load(f)
            for item in data:
                mid = int(item['id'])
                self.marker_map[mid] = np.array([float(item['x']), float(item['z']), 0.0])

        # Camera setup
        self.bridge = CvBridge()
        self.cam = Camera()
        self.cam.set_calibration('/home/pinky/ros-repo-3/roscars/aruco_mapper/camera_calibration.npz')
        self.cam.start(width=640, height=480)

        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS2 publisher for initialpose
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile=QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        self.published = False

    def capture_and_publish(self):
        frame = self.cam.get_frame()
        if frame is None:
            self.get_logger().error('No frame received')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            self.get_logger().warn('No ArUco marker detected')
            return

        marker_id = int(ids[0][0])
        if marker_id not in self.marker_map:
            self.get_logger().warn(f'Marker {marker_id} not in map')
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            0.044,  # Marker length in meters
            self.cam.calibration_matrix,
            self.cam.dist_coeffs
        )
        rvec, tvec = rvecs[0][0], tvecs[0][0]

        marker_pos_map = self.marker_map[marker_id]

        # Custom position calculation
        x = marker_pos_map[0] + tvec[2] - 0.07
        if tvec[0] >= 0:
            y = marker_pos_map[1] - tvec[0]
        else:
            y = marker_pos_map[1] + abs(tvec[0])
        z = 0.0
        base_pos_map = np.array([x, y, z])

        # Orientation
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        R_marker_cam = R_cam_marker.T
        quat = mat2quat(R_marker_cam)
        roll, pitch, yaw = mat2euler(R_marker_cam, axes='sxyz')  # ZYX 순서 기준

        # Debugging all coordinates
        self.get_logger().info(f"[DEBUG] marker_id={marker_id}")
        self.get_logger().info(f"[DEBUG] marker_pos_map: {marker_pos_map}")
        self.get_logger().info(f"[DEBUG] tvec (camera → marker): {tvec}")
        self.get_logger().info(f"[DEBUG] calculated x = marker_x + tvec_z - 0.07 = {marker_pos_map[0]} + {tvec[2]} - 0.07 = {x}")
        self.get_logger().info(f"[DEBUG] calculated y from marker_z and tvec_x: marker_z={marker_pos_map[1]}, tvec_x={tvec[0]}, result y={y}")
        self.get_logger().info(f"[DEBUG] base_pos_map (final): {base_pos_map}")
        self.get_logger().info(f"[DEBUG] R_cam_marker (rotation matrix):\n{R_cam_marker}")
        self.get_logger().info(f"[DEBUG] R_marker_cam (transpose):\n{R_marker_cam}")
        self.get_logger().info(f"[DEBUG] θ (roll)  = {np.degrees(roll):.2f} deg")
        self.get_logger().info(f"[DEBUG] θ (pitch) = {np.degrees(pitch):.2f} deg")
        self.get_logger().info(f"[DEBUG] θ (yaw)   = {np.degrees(yaw):.2f} deg")

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(base_pos_map[0])
        msg.pose.pose.position.y = float(base_pos_map[1])
        msg.pose.pose.position.z = float(base_pos_map[2])
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]
        msg.pose.covariance = [0.05] * 36

        self.pub.publish(msg)
        self.get_logger().info(f'✅ Published initialpose from marker {marker_id}: x={base_pos_map[0]:.3f}, y={base_pos_map[1]:.3f}')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    while rclpy.ok() and not node.published:
        node.capture_and_publish()
        rclpy.spin_once(node, timeout_sec=0.5)
        time.sleep(0.5)
    node.get_logger().info('Initialpose published, shutting down')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
