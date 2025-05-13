from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_sender',
            executable='camera_aruco_node',
            name='aruco_node',
            output='screen'
        ),
        Node(
            package='video_sender',
            executable='udp_streamer_node',
            name='udp_streamer',
            output='screen'
        )
    ])
