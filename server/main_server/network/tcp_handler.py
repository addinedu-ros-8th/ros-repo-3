# tcp_handler.py

import socket
import threading
from network.message_router import route_message
from logger import log_info, log_error

BUFFER_SIZE = 4096

class TCPServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_threads = []

    def start_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            log_info(f"TCP Server started on {self.host}:{self.port}")

            while True:
                client_socket, client_address = self.server_socket.accept()
                log_info(f"Accepted connection from {client_address}")

                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket,)
                )
                client_thread.start()
                self.client_threads.append(client_thread)

        except Exception as e:
            log_error(f"TCP Server Error: {str(e)}")

    def handle_client(self, client_socket):
        try:
            while True:
                data = client_socket.recv(BUFFER_SIZE)
                if not data:
                    break
                message = data.decode('utf-8')
                log_info(f"Received message: {message}")
                route_message(message, client_socket)  # 메시지를 라우터로 넘긴다
        except Exception as e:
            log_error(f"Client Handling Error: {str(e)}")
        finally:
            client_socket.close()

    def shutdown_server(self):
        if self.server_socket:
            self.server_socket.close()
        for t in self.client_threads:
            t.join()
        log_info("TCP Server shut down.")
