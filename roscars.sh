#!/bin/bash

set -e


ros2 launch pinky_bringup bringup.launch.xml
ros2 launch pinky_navigation initialpose_bringup_launch.xml 

ros2 run aruco_mapper aruco_initialpose_publisher 
ros2 run mobile_controller waypoint_follower


# Micro-ROS Agent 실행 (ESP32 실행되어있어야함)
sudo apt install ros-jazzy-micro-ros-agent
# rosdep install --from-paths src --ignore-src -r -y ## 자동 설치
echo "Launching micro_ros_agent (serial)..."
gnome-terminal -- bash -c "ros2 run micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v4; exec bash"

echo "[5] ROS2 and Micro-ROS Agent ready."
ros2 run micro_ros_agent micro_ros_agent serial   --dev /dev/ttyACM0 -b 115200 -v4



ros2 run 

