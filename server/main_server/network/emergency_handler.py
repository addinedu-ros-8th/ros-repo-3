# emergency_handler.py

from logger import log_info, log_warning, log_error

class EmergencyHandler:
    def __init__(self):
        self.is_emergency_active = False

    def trigger_emergency_stop(self, robot_id):
        """로봇에 비상 정지 명령을 트리거한다."""
        try:
            log_warning(f"[EmergencyHandler] Emergency Stop triggered for Robot ID: {robot_id}")
            self.is_emergency_active = True
            # TODO: MobileController에 EmergencyStop 명령 전송 추가 예정
        except Exception as e:
            log_error(f"[EmergencyHandler] trigger_emergency_stop Error: {str(e)}")

    def clear_emergency(self, robot_id):
        """로봇 비상 정지를 해제한다."""
        try:
            if self.is_emergency_active:
                log_info(f"[EmergencyHandler] Emergency cleared for Robot ID: {robot_id}")
                self.is_emergency_active = False
                # TODO: MobileController 정상 운영 복귀 명령 추가 예정
            else:
                log_info(f"[EmergencyHandler] No active emergency to clear for Robot ID: {robot_id}")
        except Exception as e:
            log_error(f"[EmergencyHandler] clear_emergency Error: {str(e)}")

    def handle_collision_detected(self, robot_id, sensor_data):
        """충돌 감지 시 비상 정지를 트리거한다."""
        try:
            log_warning(f"[EmergencyHandler] Collision detected by Robot ID: {robot_id}, sensor: {sensor_data}")
            self.trigger_emergency_stop(robot_id)
        except Exception as e:
            log_error(f"[EmergencyHandler] handle_collision_detected Error: {str(e)}")

    def handle_critical_battery(self, robot_id, battery_level):
        """배터리 수준이 임계치 이하일 때 비상 정지를 트리거한다."""
        try:
            if battery_level < 10:
                log_warning(f"[EmergencyHandler] Critical Battery Level ({battery_level}%) on Robot ID: {robot_id}")
                self.trigger_emergency_stop(robot_id)
        except Exception as e:
            log_error(f"[EmergencyHandler] handle_critical_battery Error: {str(e)}")
