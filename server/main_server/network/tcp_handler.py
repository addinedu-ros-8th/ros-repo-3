import socket
import threading
from logger import log_info, log_error

BUFFER_SIZE = 4096
ACCEPT_TIMEOUT = 1.0  # 소켓 accept() 타임아웃 (초)

class TCPHandler:
    def __init__(self, host, port, main_service):
        self.host = host
        self.port = port
        self.main_service = main_service
        self.server_socket = None
        self.client_threads = []
        self.is_running = False

    def soket_init(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            log_info(f"[TCPHandler] Socket initialized on {self.host}:{self.port}")
        except Exception as e:
            log_error(f"[TCPHandler] Socket Initialization Error: {str(e)}")
            raise

    def start_server(self):
        try:
            self.soket_init()
            self.is_running = True
            log_info(f"[TCPHandler] Server started on {self.host}:{self.port}")

            while self.is_running:
                try:
                    client_socket, client_address = self.server_socket.accept()
                    log_info(f"[TCPHandler] Accepted connection from {client_address}")

                    client_thread = threading.Thread(
                        target=self.handle_request,
                        args=(client_socket,)
                    )
                    client_thread.start()
                    self.client_threads.append(client_thread)

                except socket.timeout:
                    continue  # Accept timeout, continue to check if server is running

        except Exception as e:
            log_error(f"[TCPHandler] Server Error: {str(e)}")
        finally:
            self.shutdown_server()

    def handle_request(self, client_socket):
        try:
            while True:
                data = client_socket.recv(BUFFER_SIZE)
                if not data:
                    break
                message = data.decode('utf-8')
                log_info(f"[TCPHandler] Received message: {message}")
                self.main_service.route_message(message, client_socket)

        except Exception as e:
            log_error(f"[TCPHandler] Client Handling Error: {str(e)}")
        finally:
            client_socket.close()

    def shutdown_server(self):
        self.is_running = False
        if self.server_socket:
            self.server_socket.close()
            log_info("[TCPHandler] Server socket closed.")
        for t in self.client_threads:
            t.join()
        log_info("[TCPHandler] All client threads joined. Server shut down complete.")
