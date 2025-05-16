# launch/aruco_mapping.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) pinky_navigation 패키지 share 디렉토리에서 맵 YAML 동적 로드
    pkg_nav   = get_package_share_directory('pinky_navigation')
    map_yaml  = os.path.join(pkg_nav, 'map', 'roscars_map.yaml')

    # 2) aruco_mapper 패키지 share 디렉토리에서 JSON·NPZ·런치 동적 로드
    pkg_aruco = get_package_share_directory('aruco_mapper')
    marker_json = os.path.join(pkg_aruco, 'aruco_marker_positions.json')
    calib_npz   = os.path.join(pkg_aruco, 'camera_calibration.npz')

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
            }],
        ),

        # ArUco localizer
        Node(
            package='aruco_mapper',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            output='screen',
        ),

        # Marker recorder
        Node(
            package='aruco_mapper',
            executable='marker_recorder',
            name='marker_recorder',
            output='screen',
        ),

        # Initial pose publisher
        Node(
            package='aruco_mapper',
            executable='aruco_initialpose_publisher',
            name='initialpose_publisher',
            output='screen',
            parameters=[{
                'marker_map_path':    marker_json,
                'calibration_file':   calib_npz,
            }],
        ),
    ])
