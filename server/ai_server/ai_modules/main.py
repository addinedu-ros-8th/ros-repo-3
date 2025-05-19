import socket
import struct
import zlib
import cv2
import numpy as np
import time

from server.ai_server.ai_modules.object_detector import ObjectDetector, CLASS_CODE

UDP_PORT = 4436
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
        self.person_start_time = {}  # roscar_id: float
        self.person_sent = {}        # roscar_id: bool

    def receive_packet(self):
        packet, addr = self.udp_sock.recvfrom(MAX_UDP_SIZE)
        if len(packet) < 6:  # 최소 ssid_len(1) + 압축 길이(4) + 데이터(1) = 6
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

    def send_ai_result(self, ssid: str, result_code: int):
        try:
            ssid_bytes = ssid.encode('utf-8')
            ssid_len = len(ssid_bytes)
            if ssid_len > 255:
                print("[TCP 전송 실패] SSID가 너무 깁니다")
                return
            msg = struct.pack(f">2sB{ssid_len}sB", b"IN", ssid_len, ssid_bytes, result_code)
            self.tcp_sock.sendall(msg)
            print(f"[전송] IN | ssid={ssid} | result_code={result_code:#04x}")
        except Exception as e:
            print(f"[TCP 전송 실패] {e}")

    def check_and_send_person(self, ssid, results):
        now = time.time()
        frame_width = 640  # 실제 해상도에 맞게 설정

        detected_person_boxes = [
            box for box in results.boxes
            if results.names[int(box.cls[0])] == "person"
        ]

        if detected_person_boxes:
            if ssid not in self.person_start_time:
                self.person_start_time[ssid] = now
                self.person_sent[ssid] = False
            else:
                elapsed = now - self.person_start_time[ssid]
                if elapsed >= 2.0 and not self.person_sent[ssid]:
                    # ✅ 각도 계산 로직
                    for box in detected_person_boxes:
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = (x1 + x2) / 2
                        relative_x = (center_x - frame_width / 2) / (frame_width / 2)  # -1 ~ +1
                        angle = relative_x * 30  # -30도 ~ +30도 기준
                        print(f"[각도] 감지된 사람의 중심 각도: {angle:.2f}도")

                    self.send_ai_result(ssid, CLASS_CODE["Person"])
                    self.person_sent[ssid] = True
        else:
            self.person_start_time.pop(ssid, None)
            self.person_sent.pop(ssid, None)
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
                self.check_and_send_person(roscar_id, results)

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
