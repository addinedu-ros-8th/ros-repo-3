# launch/aruco_localizer.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_localizer',
            executable='aruco_localizer_node',
            name='aruco_localizer_node',
            output='screen'
        )
    ])
