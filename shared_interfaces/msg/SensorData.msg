# SensorData.msg

uint8 robot_id              # 로봇 ID
string lidar_raw            # LiDAR 데이터 (JSON 형식)
string imu_data             # IMU 데이터 (JSON 형식)
string ultrasonic_data      # 초음파 데이터 (JSON 형식)
builtin_interfaces/Time stamp  # 타임스탬프
