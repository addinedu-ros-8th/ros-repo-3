# ai_server_receiver.py
import socket
import cv2
import numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))

while True:
    data, _ = sock.recvfrom(65536)
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is not None:
        cv2.imshow("UDP Image", img)
        if cv2.waitKey(1) == 27:  # ESC 키 종료
            break
