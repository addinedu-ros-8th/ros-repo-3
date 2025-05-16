pinky_violet
==============
<img src="/doc/Screenshot from 2024-04-26 19-44-15.png" width="40%" height="30%" title="pinky" alt="pinky"></img>
# PC 설정
## 환경
- ubuntu 24.04
- ros2 jazzy
- rmw_fastrtps_cpp
## 1. pinky ROS2 pkg clone
```
mkdir -p ~/pinky_violet/src
cd ~/pinky_violet/src
git clone https://github.com/pinklab-art/pinky_violet.git
```
## 2. dependence 설치
```
cd ~/pinky_violet
rosdep install --from-paths src --ignore-src -r -y
```
## 3. build
```
cd ~/pinky_violet
colcon build
```
# ROBOT 설정
## 환경
- ubuntu 24.04
- ros2 jazzy
- rmw_fastrtps_cpp
## 1. pinky ROS2 pkg clone
```
mkdir -p ~/pinky_violet/src
cd ~/pinky_violet/src
git clone https://github.com/pinklab-art/pinky_violet.git
````
## 2. dependence 설치
```
sudo apt install python3-rpi-lgpio
sudo apt install python3-spidev
pip install smbus2
pip install adafruit-circuitpython-ads1x15
```
## 3. pinky gpio 권한 설정
#### rulse 파일 복사
```
cd ~/pinky_violet/src/pinky_violet
sudo cp ./99-pinky_gpio.rules /etc/udev/rules.d/
```
#### udev 적용
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
## 4. rplidar 설정
참고: <https://github.com/pinklab-art/pinky_violet/blob/main/doc/lidar_setup.md>

## 5. IMU 설정
참고: <https://github.com/pinklab-art/pinky_violet/blob/main/doc/imu_setup.md>

## 6. pinky_violet pkg build
```
cd ~/pinky_violet
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
## 7. set bahsrc
```
echo 'source ~/pinky_violet/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
---
Pinky brinup
-------------
```
source ~/pinky_violet/install/local_setup.bash
ros2 launch pinky_bringup bringup.launch.xml
```
or (set left_motor ratio)
```
source ~/pinky_violet/install/local_setup.bash
ros2 launch pinky_bringup bringup.launch.xml motor_ratio:=0.9
```

Map building
-------------
#### launch cartographer
```
ros2 launch pinky_cartographer cartographer.launch.xml
```
#### [ONLY PC] map view 
```
ros2 launch pinky_cartographer map_view.launch.xml
```
#### robot control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

Navigation2 
-------------
#### launch cartographer
```
ros2 launch pinky_navigation bringup_launch.xml map:=<map name>
```
or (set initial pose)
```
ros2 launch pinky_navigation initialpose_bringup_launch.xml map:=<map name>
```
#### [ONLY PC] nav2 view
```
ros2 launch pinky_navigation nav2_view.launch.xml
```

Emotion
--------------
#### start emotion server
```
ros2 run pinky_emotion pinky_emotion
```
#### set emotion
```
ros2 service call /set_emotion pinky_interfaces/srv/Emotion "{emotion: hello}"
```



