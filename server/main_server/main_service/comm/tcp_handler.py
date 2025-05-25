import socket
import threading

BUFFER_SIZE = 4096
ACCEPT_TIMEOUT = 1.0  # 소켓 accept() 타임아웃 (초)

class TCPHandler:
    def __init__(self, host, port, message_router):
        self.host = host
        self.port = port
        self.message_router = message_router
        self.server_socket = None
        self.client_threads = []
        self.is_running = False

    def soket_init(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.settimeout(ACCEPT_TIMEOUT)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            print(f"서버 실행 중: {self.host}:{self.port}")
        except Exception as e:
            print(f"소켓 초기화 실패: {e}")
            raise

    def start_server(self):
        try:
            self.soket_init()
            self.is_running = True

            while self.is_running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    print(f"클라이언트 접속: {client_address}")

                    t = threading.Thread(
                        target=self.handle_request,
                        args=(client_socket,),
                        daemon=True
                    )
                    t.start()
                    self.client_threads.append(t)

                except socket.timeout:
                    continue

        finally:
            self.shutdown_server()

    def handle_request(self, client_socket):
        try:
            while True:
                data = client_socket.recv(BUFFER_SIZE)
                if not data:
                    break

                # 원형 hex 출력
                hex_dump = ' '.join(f'{b:02X}' for b in data)
                print(f"[수신 HEX] {hex_dump}")

                # 라우터에 바이너리 그대로 전달
                self.message_router.route_message(data, client_socket)

        except Exception as e:
            print(f"클라이언트 처리 중 오류: {e}")
        finally:
            client_socket.close()
            print("클라이언트 연결 종료")

    def shutdown_server(self):
        self.is_running = False
        if self.server_socket:
            self.server_socket.close()
        for t in self.client_threads:
            t.join()
        print("서버 종료 완료")