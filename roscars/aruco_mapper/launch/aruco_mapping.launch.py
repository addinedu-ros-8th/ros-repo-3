# launch/aruco_mapping.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) pinky_navigation 패키지에서 맵 YAML 경로
    pkg_nav  = get_package_share_directory('pinky_navigation')
    map_yaml = os.path.join(pkg_nav, 'map', 'roscars_map.yaml')

    # 2) aruco_mapper 패키지에서 JSON·NPZ 파일 경로
    pkg_aruco   = get_package_share_directory('aruco_mapper')
    marker_json = os.path.join(pkg_aruco, 'aruco_marker_positions.json')
    calib_npz   = os.path.join(pkg_aruco, 'camera_calibration.npz')

    ld = LaunchDescription()

    # --- 1) map_server as LifecycleNode ---
    ld.add_entity(
        LifecycleNode(
            package='nav2_map_server',
            namespace='',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{ 'yaml_filename': map_yaml }],
        )
    )

    # --- 2) lifecycle_manager to autostart map_server ---
    ld.add_entity(
        Node(
            package='nav2_lifecycle_manager',
            namespace='',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                { 'use_sim_time': False },
                { 'autostart': True },
                { 'bond_timeout': 0.0 },
                { 'node_names': ['map_server'] },
            ],
        )
    )

    # --- 3) Static transform: map → base_link ---
    ld.add_entity(
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='static_map_to_base_link',
            arguments=[ '0', '0', '0', '0', '0', '0', 'map', 'base_link' ],
            output='screen',
        )
    )

    # --- 4) Camera publisher ---
    ld.add_entity(
        Node(
            package='aruco_mapper',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
        )
    )

    # --- 5) ArUco localizer ---
    ld.add_entity(
        Node(
            package='aruco_mapper',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            output='screen',
            parameters=[{ 'calibration_file': calib_npz }],
        )
    )

    # --- 6) Marker recorder ---

    # --- 7) Initial pose publisher ---
    ld.add_entity(
        Node(
            package='aruco_mapper',
            executable='aruco_initialpose_publisher',
            name='initialpose_publisher',
            output='screen',
            parameters=[
                { 'marker_map_path':  marker_json },
                { 'calibration_file': calib_npz },
            ],
        )
    )

    return ld
