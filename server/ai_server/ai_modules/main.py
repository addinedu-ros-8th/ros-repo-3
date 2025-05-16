import socket
import struct
import zlib
import cv2
import numpy as np

from server.ai_server.ai_modules.object_detector import ObjectDetector, CLASS_CODE

UDP_PORT = 4436
# TCP_IP = "192.168.0.30"
TCP_IP = "localhost"
TCP_PORT = 5001
MAX_UDP_SIZE = 65507

class AIModules:
    def __init__(self):
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('0.0.0.0', UDP_PORT))
        print(f"[AI] UDP 수신 대기 중 (포트 {UDP_PORT})")

        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_sock.connect((TCP_IP, TCP_PORT))
        print(f"[AI] TCP 연결 완료 ({TCP_IP}:{TCP_PORT})")

        self.detector = ObjectDetector()

    def receive_packet(self):
        packet, addr = self.udp_sock.recvfrom(MAX_UDP_SIZE)
        if len(packet) < 5:
            return None, None

        roscar_id, comp_len = struct.unpack('!BI', packet[:5])
        comp = packet[5:]

        if len(comp) != comp_len:
            return None, None

        try:
            decompressed = zlib.decompress(comp)
        except zlib.error:
            return None, None

        return roscar_id, decompressed

    def process_frame(self, data):
        np_data = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        return frame

    def send_result(self, roscar_id, results):
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id]
            if cls_name not in CLASS_CODE:
                continue

            result_code = CLASS_CODE[cls_name]
            binary_msg = struct.pack('>2sBB', b'IN', roscar_id, result_code)
            try:
                self.tcp_sock.sendall(binary_msg)
                print(f"[전송] IN | ID={roscar_id} | class={cls_name}({result_code})")
            except Exception as e:
                print("TCP 전송 실패:", e)

    def run(self):
        try:
            while True:
                roscar_id, decompressed = self.receive_packet()
                if roscar_id is None:
                    continue

                frame = self.process_frame(decompressed)
                if frame is None:
                    continue

                results = self.detector.on_frame_received(frame, roscar_id)
                self.send_result(roscar_id, results)

                # 시각화 (디버그용)
                annotated = results.plot()
                cv2.imshow(f"Roscar_{roscar_id}", annotated)
                if cv2.waitKey(1) == 27:
                    break

        except KeyboardInterrupt:
            print("[AI] 종료 요청 수신")
        finally:
            self.udp_sock.close()
            self.tcp_sock.close()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    AIModules().run()
