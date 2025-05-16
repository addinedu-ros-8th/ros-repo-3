# viewer/staff/tcp_sender.py

from PyQt6.QtCore import QThread, pyqtSignal
import socket
import struct

class TCPClientThread(QThread):
    received = pyqtSignal(bytes)  # 이제 바이너리 응답도 받을 수 있게

    def __init__(self, host='127.0.0.1', port=9000):
        super().__init__()
        self.host = host
        self.port = port
        self.socket = None
        self._running = True
        self._message_to_send = None

    def run(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(0.5)
        except Exception as e:
            print(f"❌ 서버 연결 실패: {e}")
            return

        try:
            while self._running:
                # 송신
                if self._message_to_send is not None:
                    if isinstance(self._message_to_send, str):
                        payload = self._message_to_send.encode('utf-8')
                    else:
                        payload = self._message_to_send
                    try:
                        self.socket.sendall(payload)
                    except Exception as e:
                        print(f"❌ 전송 에러: {e}")
                        break
                    finally:
                        self._message_to_send = None

                # 수신
                try:
                    data = self.socket.recv(4096)
                    if data:
                        self.received.emit(data)
                except socket.timeout:
                    continue

        finally:
            if self.socket:
                self.socket.close()

    def send(self, msg: bytes | str):
        """바이트 혹은 문자열을 보내도록 설정"""
        self._message_to_send = msg

    # ── Protocol 모듈화 메서드 ──

    def send_login_request(self, user_name: str, password: str):
        # [AU] 로그인 인증 요청
        cmd = b"AU"
        u = user_name.encode('utf-8')[:32].ljust(32, b'\x00')
        p = password.encode('utf-8')[:32].ljust(32, b'\x00')
        self.send(cmd + u + p)

    def send_item_info_request(self, qr_code: str):
        # [IS] 상품 정보 조회 요청
        cmd = b"IS"
        q = qr_code.encode('utf-8')[:16].ljust(16, b'\x00')
        self.send(cmd + q)

    def send_inventory_request(self, user_id: int, destination: str, items: list[dict]):
        # [IR] 장바구니 기반 상품 요청
        cmd = b"IR"
        header = (
            struct.pack('>I', user_id) +
            struct.pack('>H', len(items)) +
            destination.encode('utf-8')[:2]
        )
        body = b""
        for it in items:
            # shoes_model_id, location_id, quantity 순서
            body += struct.pack('>III',
                                it['shoes_model_id'],
                                it['location_id'],
                                it['quantity'])
        self.send(cmd + header + body)

    def send_cancel_delivery_request(self, user_id: int, delivery_id: int):
        # [CD] 배송 취소 요청
        cmd = b"CD"
        payload = struct.pack('>I I', user_id, delivery_id)
        self.send(cmd + payload)

    def send_task_status_request(self, user_id: int):
        # [TR] 작업 상태 확인 요청
        cmd = b"TR"
        self.send(cmd + struct.pack('>I', user_id))

    def stop(self):
        self._running = False
        self.wait()
