# test_tcp_handler.py

import sys
import os
import threading
import time
import socket

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from network.tcp_handler import TCPServer

# Mock MainService (TCPServer는 main_service 필요)
class MockMainService:
    def route_message(self, message, client_socket):
        print(f"[MockMainService] route_message called with message: {message}")
        # 테스트용으로 바로 에코 응답
        response = {"type": "TestResponse", "message": "Echo"}
        client_socket.sendall(str(response).encode('utf-8'))

def start_server():
    server = TCPServer(host="127.0.0.1", port=5050, main_service=MockMainService())
    server_thread = threading.Thread(target=server.start_server)
    server_thread.start()
    return server, server_thread

def test_client_send_message():
    try:
        print("[Test] Connecting to server...")
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(("127.0.0.1", 5050))
        
        test_message = '{"type": "TestRequest", "content": "Hello Server"}'
        print(f"[Test] Sending message: {test_message}")
        client_socket.sendall(test_message.encode('utf-8'))

        # 응답 수신
        response = client_socket.recv(4096)
        print(f"[Test] Received response: {response.decode('utf-8')}")

        client_socket.close()
    except Exception as e:
        print(f"[Test] Client error: {e}")

if __name__ == "__main__":
    server, server_thread = start_server()

    # 서버가 완전히 뜰 때까지 잠깐 기다림
    time.sleep(1)

    test_client_send_message()

    # 서버 종료
    print("[Test] Shutting down server...")
    server.shutdown_server()
    server_thread.join()
    print("[Test] Server shutdown complete.")
