# launch.py

import os
import subprocess
from logger import log_info, log_error

class ROSNodeLauncher:
    def __init__(self):
        # ROS2 환경 설정
        self.ros_domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
        self.ros_namespace = "/roscar"

    def launch_navigation_node(self):
        try:
            log_info("[ROSNodeLauncher] Launching Navigation Node...")
            subprocess.Popen(["ros2", "launch", "roscar_navigation", "navigation.launch.py"])
        except Exception as e:
            log_error(f"[ROSNodeLauncher] Failed to launch navigation node: {str(e)}")

    def launch_bridge_node(self):
        try:
            log_info("[ROSNodeLauncher] Launching ROS-Gazebo Bridge Node...")
            subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/model/roscar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"])
        except Exception as e:
            log_error(f"[ROSNodeLauncher] Failed to launch bridge node: {str(e)}")

    def launch_custom_node(self, package_name, executable_name):
        try:
            log_info(f"[ROSNodeLauncher] Launching custom node: {package_name} {executable_name}...")
            subprocess.Popen(["ros2", "run", package_name, executable_name])
        except Exception as e:
            log_error(f"[ROSNodeLauncher] Failed to launch custom node {package_name}/{executable_name}: {str(e)}")

