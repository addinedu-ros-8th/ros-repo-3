from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([

        # Domain 1: roscar1_listener 실행
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'main_server_pkg', 'multi_domain_listener'
            ],
            name='roscar1_listener',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '25'}
        ),

        # Domain 2: roscar2_listener 실행
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'main_server_pkg', 'multi_domain_listener'
            ],
            name='roscar2_listener',
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '36'}
        ),

        # # Domain 3: roscar3_listener 실행
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'run', 'main_server_pkg', 'multi_domain_listener'
        #     ],
        #     name='roscar3_listener',
        #     output='screen',
        #     additional_env={'ROS_DOMAIN_ID': '3'}
        # ),

    ])
