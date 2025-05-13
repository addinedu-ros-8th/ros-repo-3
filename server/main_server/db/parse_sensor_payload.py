# db/parse_sensor_payload.py
import json

def parse_sensor_data(msg):
    """
    SensorData 메시지로부터 LiDAR, IMU, 초음파 JSON을 파싱하여 dict로 반환
    """
    lidar = json.loads(msg.lidar_raw)
    imu = json.loads(msg.imu_data)
    ultra = json.loads(msg.ultrasonic_data)

    return {
        "lidar": lidar,
        "imu": imu,
        "ultra": ultra
    }
