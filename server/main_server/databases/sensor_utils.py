def parse_sensor_data(msg):
    return {
        "lidar": {
            "ranges": list(msg.lidar_ranges),
            "angle_min": msg.lidar_angle_min,
            "angle_max": msg.lidar_angle_max
        },
        "imu": {
            "accel": {
                "x": msg.imu_accel.x,
                "y": msg.imu_accel.y,
                "z": msg.imu_accel.z
            },
            "gyro": {
                "x": msg.imu_gyro.x,
                "y": msg.imu_gyro.y,
                "z": msg.imu_gyro.z
            },
            "mag": {
                "x": msg.imu_mag.x,
                "y": msg.imu_mag.y,
                "z": msg.imu_mag.z
            }
        },
        "ultra": {
            "front": msg.ultrasonic_front,
            "left": msg.ultrasonic_left,
            "right": msg.ultrasonic_right
        }
    }
