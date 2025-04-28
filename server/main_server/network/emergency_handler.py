# emergency_handler.py

from logger import log_info, log_warning, log_error

class EmergencyHandler:
    def __init__(self):
        self.is_emergency_active = False

    def trigger_emergency_stop(self, robot_id):
        log_warning(f"[EmergencyHandler] Emergency Stop triggered for Robot ID: {robot_id}")
        self.is_emergency_active = True
        # TODO: MobileController에게 EmergencyStop 명령 보내는 로직 추가 예정

    def clear_emergency(self, robot_id):
        if self.is_emergency_active:
            log_info(f"[EmergencyHandler] Emergency cleared for Robot ID: {robot_id}")
            self.is_emergency_active = False
            # TODO: Emergency 해제 후 정상 운영 복귀 명령 보내는 로직 추가 예정
        else:
            log_info(f"[EmergencyHandler] No active emergency to clear for Robot ID: {robot_id}")

    def handle_collision_detected(self, robot_id, sensor_data):
        log_warning(f"[EmergencyHandler] Collision detected by Robot ID: {robot_id}, sensor: {sensor_data}")
        self.trigger_emergency_stop(robot_id)

    def handle_critical_battery(self, robot_id, battery_level):
        if battery_level < 10:
            log_warning(f"[EmergencyHandler] Critical Battery Level ({battery_level}%) on Robot ID: {robot_id}")
            self.trigger_emergency_stop(robot_id)

