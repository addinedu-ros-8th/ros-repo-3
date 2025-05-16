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
from transforms3d.euler import mat2euler  # for roll, pitch, yaw

class ArucoInitialPosePublisher(Node):
    EAST_MARKER_IDS  = {2, 3, 13, 14, 15, 16, 17, 18, 28, 29, 30, 34}
    WEST_MARKER_IDS  = {0, 5, 6, 7, 8, 20, 22, 23, 24, 32}
    SOUTH_MARKER_IDS = {1, 4, 11, 19, 21, 25, 26, 27, 31, 33, 35}
    NORTH_MARKER_IDS = {9, 10, 12, 25, 26, 27, 33}
    OTHERS_MARKER_IDS= {36, 37, 38, 39}

    def __init__(self):
        super().__init__('aruco_initialpose_publisher')

        # Load marker map from JSON
        self.marker_map = {}
        json_path = '/home/pinky/ros-repo-3/roscars/aruco_mapper/aruco_marker_positions.json'
        with open(json_path, encoding='utf-8') as f:
            data = json.load(f)
        for item in data:
            mid = int(item['id'])
            # JSON has 'x' and 'y' fields for map coords
            self.marker_map[mid] = np.array([
                float(item['x']),
                float(item['y']),
                0.0
            ])

        # Camera setup
        self.bridge = CvBridge()
        self.cam = Camera()
        self.cam.set_calibration("/home/pinky/ros-repo-3/roscars/aruco_mapper/camera_calibration.npz")
        self.cam.start(width=640, height=480)

        # ArUco detector setup
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS2 publisher for initialpose (latched)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.published = False

    def capture_and_publish(self):
        frame = self.cam.get_frame()
        if frame is None:
            self.get_logger().error('No frame received')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        if ids is None or len(ids) == 0:
            self.get_logger().warn('No ArUco marker detected')
            return

        marker_id = int(ids[0][0])
        if marker_id not in self.marker_map:
            self.get_logger().warn(f'Marker {marker_id} not in map')
            return

        # Estimate pose (marker_size=0.044 m)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            0.044,
            self.cam.calibration_matrix,
            self.cam.dist_coeffs
        )
        rvec, tvec = rvecs[0][0], tvecs[0][0]
        mpos = self.marker_map[marker_id]  # [x_map, y_map, 0]

        # ------ Replace multiple ifs with single if/elif chain ------
        if marker_id in self.EAST_MARKER_IDS:
            # East wall
            x = mpos[0] + tvec[2] + 0.07
            y = mpos[1] - abs(tvec[0]) if tvec[0] >= 0 else mpos[1] + abs(tvec[0])
        elif marker_id in self.SOUTH_MARKER_IDS:
            # South wall
            y = mpos[1] - abs(tvec[2]) - 0.07
            x = mpos[0] - abs(tvec[0]) if tvec[0] >= 0 else mpos[0] + abs(tvec[0])
        elif marker_id in self.WEST_MARKER_IDS:
            # West wall
            x = mpos[0] - abs(tvec[2]) - 0.07
            y = mpos[1] + abs(tvec[0]) if tvec[0] >= 0 else mpos[1] - abs(tvec[0])
        elif marker_id in self.NORTH_MARKER_IDS:
            # North wall
            y = mpos[1] + abs(tvec[2]) + 0.07
            x = mpos[0] + abs(tvec[0]) if tvec[0] >= 0 else mpos[0] - abs(tvec[0])
        else:
            # Others: no special offset
            x, y = mpos[0], mpos[1]

        base_pos_map = np.array([x, y, 0.0])
        # ------------------------------------------------------------

        # Compute orientation: camera→marker rotation → inverse
        R_cam_marker, _ = cv2.Rodrigues(rvec)
        R_marker_cam = R_cam_marker.T
        quat = mat2quat(R_marker_cam)
        roll, pitch, yaw = mat2euler(R_marker_cam, axes='sxyz')

        # Debug logs
        self.get_logger().info(f"[DEBUG] marker_id={marker_id}")
        self.get_logger().info(f"[DEBUG] map_pos   = {mpos}")
        self.get_logger().info(f"[DEBUG] tvec      = {tvec}")
        self.get_logger().info(f"[DEBUG] base_pos  = {base_pos_map}")
        self.get_logger().info(f"[DEBUG] quat      = {quat}")
        self.get_logger().info(
            f"[DEBUG] r,p,y    = {np.degrees(roll):.1f},"
            f"{np.degrees(pitch):.1f},"
            f"{np.degrees(yaw):.1f} deg"
        )

        # Build and publish PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x    = float(base_pos_map[0])
        msg.pose.pose.position.y    = float(base_pos_map[1])
        msg.pose.pose.position.z    = float(base_pos_map[2])
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]
        msg.pose.covariance = [0.05] * 36

        self.pub.publish(msg)
        self.get_logger().info(
            f"✅ Published initialpose from marker {marker_id}: "
            f"x={base_pos_map[0]:.3f}, y={base_pos_map[1]:.3f}"
        )
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    try:
        while rclpy.ok() and not node.published:
            node.capture_and_publish()
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(0.5)
        node.get_logger().info('Initialpose published, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
