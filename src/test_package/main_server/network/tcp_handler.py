import socket
import threading
class TCPHandler:
    def __init__(self, host='0.0.0.0', port=8888):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print(f'TCP server listening on {self.host}:{self.port}')
    def handle_client(self, client_socket, address):
        print(f'Accepted connection from {address}')
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                print(f'Received from {address}: {data.decode()}')
                client_socket.sendall(b'ACK')
        except ConnectionResetError:
            print(f'Connection reset by {address}')
        finally:
            client_socket.close()
            print(f'Connection closed for {address}')
    def start(self):
        while True:
            client_sock, addr = self.server_socket.accept()
            thread = threading.Thread(target=self.handle_client, args=(client_sock, addr))
            thread.start()
if __name__ == '__main__':
    server = TCPHandler()
    server.start()