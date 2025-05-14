from task_manager import TaskManager

class MainService:
    def __init__(self):
        super().__init__('main_service_node')
        self.task_manager = TaskManager()
        # ROS2 pub/sub, service setup 등

    def handle_login_request(self, data, client_socket):
        """LoginRequest 처리"""
        pass

    def handle_create_task_request(self, data, client_socket):
        """CreateTaskRequest 처리"""
        pass

    def handle_qrcode_search(self, data, client_socket):
        """SearchQRCodeData 처리"""
        pass

    def handle_roscar_status_request(self, data, client_socket):
        """RequestRoscarStatus 처리"""
        pass

    def handle_inventory_request(self, data, client_socket):
        """InventoryRequest 처리"""
        pass

    def handle_register_roscar(self, data, client_socket):
        """RegisterRoscarRequest 처리"""
        pass

    def handle_delete_roscar(self, data, client_socket):
        """DeleteRoscarRequest 처리"""
        pass

    def handle_location_request(self, data, client_socket):
        """RequestRoscarLocation 처리"""
        pass

    def handle_log_request(self, data, client_socket):
        """RequestLogs 처리"""
        pass

    # AI 서버로부터 응급 정지 등 수신 시 대비 (옵션)
    def handle_emergency_stop(self, data, client_socket):
        """EmergencyStop 처리 (추후 확장용)"""
        pass
