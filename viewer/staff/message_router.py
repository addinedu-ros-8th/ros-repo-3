import struct
from PyQt6.QtWidgets import QMessageBox

class MessageRouter:
    def __init__(self, main_service=None, parent_gui=None):
        self.main_service = main_service
        self.parent_gui = parent_gui

    def handle_response(self, data: bytes):
        if len(data) < 3:
            print("[Router] 응답 너무 짧음")
            return

        cmd = data[:2]
        status = data[2]

        if cmd == b"AU":
            self._handle_au_response(data, status)
        elif cmd == b"IS":
            self._handle_is_response(data, status)
        elif cmd == b"IR":
            self._handle_ir_response(data, status)
        elif cmd == b"CD":
            self._handle_cd_response(data, status)
        elif cmd == b"TR":
            self._handle_tr_response(data, status)
        else:
            print(f"[Router] 알 수 없는 명령어: {cmd}")

    def _handle_au_response(self, data: bytes, status: int):
        if status == 0x00:
            if len(data) < 8:
                print("[Router] AU 응답 길이 부족")
                return
            user_id = struct.unpack('>I', data[3:7])[0]
            user_role = data[7]
            print(f"[Router] 로그인 성공: ID={user_id}, ROLE={user_role}")
            self.parent_gui.on_login_success(user_id, user_role)
        else:
            QMessageBox.critical(None, "로그인 실패", "사용자 이름 또는 비밀번호가 일치하지 않습니다.")

    def _handle_is_response(self, data: bytes, status: int):
        if status == 0x00:
            if len(data) < 75:
                print("[Router] IS 응답 길이 부족")
                return

            name = data[3:35].decode('utf-8').rstrip('\x00')
            size = struct.unpack('<I', data[35:39])[0]
            color = data[39:55].decode('utf-8').rstrip('\x00')
            quantity = struct.unpack('<I', data[55:59])[0]
            location = data[59:75].decode('utf-8').rstrip('\x00')

            product_data = {
                "qr_data": self.parent_gui.camera_panel.last_detected_qr,
                "model": name,
                "size": str(size),
                "color": color,
                "rack": location,
                "quantity": quantity
            }

            self.parent_gui.go_to_product_info(product_data)

        elif status == 0x01:
            QMessageBox.warning(None, "안내", "일치하는 재고가 없습니다.")
            self.parent_gui.camera_panel.reset_qr_detection()
        else:
            print("[Router] 알 수 없는 상태 코드")

    def _handle_ir_response(self, data: bytes, status: int):
        if status == 0x00:
            if len(data) < 11:
                print("[Router] IR 응답 길이 부족")
                return
            delivery_id = struct.unpack('>I', data[3:7])[0]
            task_id = struct.unpack('>I', data[7:11])[0]
            print(f"[Router] 요청 성공: 배송 ID={delivery_id}, 첫 Task ID={task_id}")
            
            # ✅ 작업 상태 패널로 이동
            if hasattr(self.parent_gui, "request_wait_panel"):
                self.parent_gui.request_wait_panel.on_inventory_response_success()

        else:
            QMessageBox.critical(None, "요청 실패", "배송 요청 처리에 실패했습니다.")

    def _handle_cd_response(self, data: bytes, status: int):
        if len(data) < 7:
            print("[Router] CD 응답 길이 부족")
            return
        delivery_id = struct.unpack('>I', data[3:7])[0]
        if status == 0x00:
            QMessageBox.information(None, "배송 취소", f"배송 ID {delivery_id}가 취소되었습니다.")
            self.parent_gui.on_cancel_success(delivery_id)
        else:
            QMessageBox.warning(None, "취소 실패", "배송 상태 때문에 취소할 수 없습니다.")

    def _handle_tr_response(self, data: bytes, status: int):
        if status == 0x00:
            if len(data) < 6:
                print("[Router] TR 응답 길이 부족")
                return
            done_count = struct.unpack('>H', data[3:5])[0]
            in_progress_count = data[5]
            names = []
            offset = 6
            for _ in range(in_progress_count):
                name = data[offset:offset + 32].decode('utf-8').rstrip('\x00')
                names.append(name)
                offset += 32

            self.parent_gui.show_task_status(done_count, names)
        else:
            QMessageBox.warning(None, "작업 조회 실패", "이전 작업을 조회할 수 없습니다.")
