import socket
import struct
import zlib
import cv2
import numpy as np
import time

from server.ai_server.ai_modules.object_detector import ObjectDetector, CLASS_CODE

UDP_PORT = 4436
TCP_IP = "127.0.0.1"
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
        self.detection_start_time = {}  # key=(ssid, class_name), value=start_time
        self.detection_sent = {}        # key=(ssid, class_name), value=bool

    def receive_packet(self):
        packet, addr = self.udp_sock.recvfrom(MAX_UDP_SIZE)
        if len(packet) < 6:
            return None, None

        ssid_len = packet[0]
        if len(packet) < 1 + ssid_len + 4:
            return None, None

        ssid = packet[1:1+ssid_len].decode('utf-8')
        comp_len = struct.unpack('!I', packet[1+ssid_len:5+ssid_len])[0]
        comp = packet[5+ssid_len:]

        if len(comp) != comp_len:
            print(f"[❌] 압축 길이 불일치: 수신={len(comp)}, 명시된={comp_len}")
            return None, None

        try:
            decompressed = zlib.decompress(comp)
        except zlib.error:
            print("[❌] 압축 해제 실패")
            return None, None

        return ssid, decompressed

    def process_frame(self, data):
        np_data = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        if frame is None:
            print("[AI] 프레임 디코딩 실패")
            return None

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, 0)
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

    def send_ai_result(self, ssid: str, result_code: int, angle: float):
        try:
            ssid_bytes = ssid.encode('utf-8')
            ssid_len = len(ssid_bytes)
            if ssid_len > 255:
                print("[TCP 전송 실패] SSID가 너무 깁니다")
                return
            msg = struct.pack(f">2sB{ssid_len}sBf", b"IN", ssid_len, ssid_bytes, result_code, angle)
            self.tcp_sock.sendall(msg)
            print(f"[전송] IN | ssid={ssid} | result_code={result_code:#04x} | angle={angle:.2f}")
        except Exception as e:
            print(f"[TCP 전송 실패] {e}")

    def check_and_send_target(self, ssid, results):
        now = time.time()
        frame_width = 640
        target_classes = ["person", "roscar"]

        for target in target_classes:
            boxes = [
                box for box in results.boxes
                if results.names[int(box.cls[0])] == target
            ]

            key = (ssid, target)

            if boxes:
                if key not in self.detection_start_time:
                    self.detection_start_time[key] = now
                    self.detection_sent[key] = False
                else:
                    elapsed = now - self.detection_start_time[key]
                    if elapsed >= 2.0 and not self.detection_sent[key]:
                        for box in boxes:
                            x1, _, x2, _ = box.xyxy[0]
                            center_x = (x1 + x2) / 2
                            relative_x = (center_x - frame_width / 2) / (frame_width / 2)
                            angle = relative_x * 30  # -30 ~ +30

                            result_code = CLASS_CODE[target.capitalize()]
                            self.send_ai_result(ssid, result_code, angle)
                            print(f"[전송] {target} → angle={angle:.2f}도")
                            break
                        self.detection_sent[key] = True
            else:
                self.detection_start_time.pop(key, None)
                self.detection_sent.pop(key, None)

    def run(self):
        try:
            while True:
                ssid, decompressed = self.receive_packet()
                if ssid is None:
                    continue

                frame = self.process_frame(decompressed)
                if frame is None:
                    continue

                results = self.detector.on_frame_received(frame, ssid)
                self.send_result(ssid, results)
                self.check_and_send_target(ssid, results)

                annotated = results.plot()
                cv2.imshow(f"Roscar_{ssid}", annotated)
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
