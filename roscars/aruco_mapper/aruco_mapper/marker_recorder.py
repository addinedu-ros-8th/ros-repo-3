#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import numpy as np
import csv
import os
from transforms3d.quaternions import quat2mat
from transforms3d.affines import compose

class MarkerRecorder(Node):
    def __init__(self):
        super().__init__('marker_recorder')

        # 1) MarkerArray 구독
        self.subscription = self.create_subscription(
            MarkerArray,
            '/aruco_markers',
            self.marker_callback,
            10
        )

        # 2) TF listener (map ↔ base_link)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3) base_link → camera_link 고정 변환 (카메라 높이 6cm)
        self.T_base2cam = np.eye(4, dtype=float)
        self.T_base2cam[2, 3] = 0.06

        # 4) 결과 CSV 파일 준비
        output_dir = os.path.expanduser('~/ros-repo-3/roscars/aruco_mapper')
        os.makedirs(output_dir, exist_ok=True)
        self.output_file = os.path.join(output_dir, 'aruco_marker_positions.csv')
        with open(self.output_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['marker_id', 'x', 'y', 'z'])

        self.get_logger().info(f"Saving to {self.output_file}")

    def marker_callback(self, msg: MarkerArray):
        # (A) map → base_link TF 먼저 가져오기
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            trans = [t.x, t.y, t.z]
            quat  = [r.w, r.x, r.y, r.z]
            R_map2base = quat2mat(quat)
            T_map2base = compose(trans, R_map2base, [1, 1, 1])
            T_map2cam  = T_map2base @ self.T_base2cam

        except Exception as e:
            self.get_logger().warn(f"TF map→base_link failed: {e}")
            return

        # (B) 각 마커별로 한 번만 기록
        for marker in msg.markers:
            mid = marker.id
            if mid in getattr(self, 'saved_ids', set()):
                continue

            # 카메라 프레임에서 온 tvec (x,y,z)
            tvec_cam = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
                1.0
            ], dtype=float)

            # map frame 절대 좌표 계산
            xyz_map = (T_map2cam @ tvec_cam)[:3]
            x, y, z = xyz_map.tolist()

            # CSV에 append
            with open(self.output_file, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([mid, x, y, z])

            self.get_logger().info(
                f"[Saved] Marker {mid}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

            # 한 번만 저장
            if not hasattr(self, 'saved_ids'):
                self.saved_ids = set()
            self.saved_ids.add(mid)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
