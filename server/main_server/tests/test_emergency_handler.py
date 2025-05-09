# test_emergency_handler.py

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from network.emergency_handler import EmergencyHandler

def test_emergency_stop_and_clear():
    try:
        handler = EmergencyHandler()

        print("Testing trigger_emergency_stop() ...")
        handler.trigger_emergency_stop("roscar_001")

        print("Testing clear_emergency() ...")
        handler.clear_emergency("roscar_001")

    except Exception as e:
        print(f"Unexpected error in test_emergency_stop_and_clear: {e}")

def test_collision_detected():
    try:
        handler = EmergencyHandler()

        print("Testing handle_collision_detected() ...")
        handler.handle_collision_detected("roscar_002", sensor_data="Front_Lidar")

    except Exception as e:
        print(f"Unexpected error in test_collision_detected: {e}")

def test_critical_battery():
    try:
        handler = EmergencyHandler()

        print("Testing handle_critical_battery() ...")
        handler.handle_critical_battery("roscar_003", battery_level=5)  # 낮은 배터리(임계치 이하)

        print("Testing handle_critical_battery() with normal battery ...")
        handler.handle_critical_battery("roscar_003", battery_level=50)  # 정상 배터리

    except Exception as e:
        print(f"Unexpected error in test_critical_battery: {e}")

if __name__ == "__main__":
    test_emergency_stop_and_clear()
    print("-" * 40)
    test_collision_detected()
    print("-" * 40)
    test_critical_battery()
