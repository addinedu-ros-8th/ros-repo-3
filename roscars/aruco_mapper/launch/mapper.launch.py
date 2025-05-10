from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_mapper',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            output='screen'
        ),
        Node(
            package='aruco_mapper',
            executable='marker_recorder',
            name='marker_recorder',
            output='screen'
        ),
    ])
