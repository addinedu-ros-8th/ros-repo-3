from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QMessageBox
import struct

class MessageRouter(QObject):
    # 시그널 정의
    loginSuccess         = pyqtSignal(int, int)    # user_id, user_role_code
    loginFailure         = pyqtSignal()
    inventoryInfo        = pyqtSignal(dict)        # product_data
    inventoryNotFound    = pyqtSignal()
    inventoryRequestSuccess = pyqtSignal()
    inventoryRequestFailure = pyqtSignal()
    cancelSuccess        = pyqtSignal(int)         # delivery_id
    cancelFailure        = pyqtSignal(int)         # delivery_id
    taskStatusReceived   = pyqtSignal(int, list)   # done_count, [(delivery_id, model_name), ...]
    taskStatusFailure    = pyqtSignal()

    def __init__(self, parent_gui=None):
        super().__init__(parent_gui)
        self.parent_gui = parent_gui

    def handle_response(self, data: bytes):
        if len(data) < 3:
            print("[Router] 응답 너무 짧음")
            return

        cmd, status = data[:2], data[2]

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
        if status == 0x00 and len(data) >= 8:
            user_id   = struct.unpack('>I', data[3:7])[0]
            user_role = data[7]
            self.loginSuccess.emit(user_id, user_role)
            if self.parent_gui and hasattr(self.parent_gui, 'on_login_success'):
                self.parent_gui.on_login_success(user_id, user_role)
        else:
            self.loginFailure.emit()
            QMessageBox.critical(None, "로그인 실패", "사용자 이름 또는 비밀번호가 일치하지 않습니다.")

    def _handle_is_response(self, data: bytes, status: int):
        if status == 0x00 and len(data) >= 75:
            name     = data[3:35].decode('utf-8').rstrip('\x00')
            size     = struct.unpack('<I', data[35:39])[0]
            color    = data[39:55].decode('utf-8').rstrip('\x00')
            quantity = struct.unpack('<I', data[55:59])[0]
            location = data[59:75].decode('utf-8').rstrip('\x00')
            product_data = {
                'qr_data': getattr(self.parent_gui, 'camera_panel', None) and self.parent_gui.camera_panel.last_detected_qr,
                'model':   name,
                'size':    size,
                'color':   color,
                'rack':    location,
                'quantity':quantity
            }
            self.inventoryInfo.emit(product_data)
            if self.parent_gui and hasattr(self.parent_gui, 'go_to_product_info'):
                self.parent_gui.go_to_product_info(product_data)
        elif status == 0x01:
            self.inventoryNotFound.emit()
            QMessageBox.warning(None, "안내", "일치하는 재고가 없습니다.")
            if self.parent_gui and hasattr(self.parent_gui, 'camera_panel'):
                self.parent_gui.camera_panel.reset_qr_detection()
        else:
            print("[Router] IS 응답 처리 실패")

    def _handle_ir_response(self, data: bytes, status: int):
        if status == 0x00 and len(data) >= 11:
            self.inventoryRequestSuccess.emit()
            if self.parent_gui and hasattr(self.parent_gui, 'request_wait_panel'):
                self.parent_gui.request_wait_panel.on_inventory_response_success()
        else:
            self.inventoryRequestFailure.emit()
            QMessageBox.critical(None, "요청 실패", "배송 요청 처리에 실패했습니다.")

    def _handle_cd_response(self, data: bytes, status: int):
        # 최소 7바이트 확보 (2B cmd + 1B status + 4B delivery_id)
        if len(data) < 7:
            print("[Router] CD 응답 처리 실패: 데이터 길이 부족")
            return

        delivery_id = struct.unpack('>I', data[3:7])[0]

        if status == 0x00:
            self.cancelSuccess.emit(delivery_id)
            if self.parent_gui and hasattr(self.parent_gui, 'on_cancel_success'):
                self.parent_gui.on_cancel_success(delivery_id)
        else:
            self.cancelFailure.emit(delivery_id)

    def _handle_tr_response(self, data: bytes, status: int):
        # 최소 6바이트 확보 (2B cmd + 1B status + 2B done_count + 1B in_progress_count)
        if status == 0x00 and len(data) >= 6:
            # 완료된 Task 수: uint16
            done_count  = struct.unpack_from(">H", data, 3)[0]
            # 진행 중 Task 개수: uint8
            in_prog_cnt = data[5]

            # 각 진행 중 항목은 4바이트 delivery_id + 32바이트 모델명
            expected = 6 + in_prog_cnt * (4 + 32)
            if len(data) < expected:
                print(f"[Router] TR 바이트 부족: {len(data)}/{expected}")
                return

            items = []
            offset = 6
            for _ in range(in_prog_cnt):
                did = struct.unpack_from(">I", data, offset)[0]
                offset += 4
                name = data[offset:offset+32].decode("utf-8").rstrip("\x00")
                offset += 32
                items.append((did, name))

            # 시그널로 (done_count, [(delivery_id, name), ...]) 전달
            self.taskStatusReceived.emit(done_count, items)
            if self.parent_gui and hasattr(self.parent_gui, 'show_task_status'):
                self.parent_gui.show_task_status(done_count, items)
        else:
            self.taskStatusFailure.emit()
            QMessageBox.warning(None, "작업 조회 실패", "이전 작업을 조회할 수 없습니다.")