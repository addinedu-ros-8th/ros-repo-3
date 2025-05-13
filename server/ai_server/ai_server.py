import socket
import cv2
import numpy as np
import struct

MAX_UDP_SIZE = 65507  # UDP 최대 페이로드

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 4436))
    print("AI 서버 UDP 수신 대기 중 (포트 4436)...")

    try:
        while True:
            packet, addr = sock.recvfrom(MAX_UDP_SIZE)
            print(f"수신됨: {len(packet)} bytes from {addr}")

            if len(packet) < 5:
                print("⚠️ 헤더 미달, 스킵")
                continue

            # 1B robot_id, 4B frame_size
            robot_id, frame_size = struct.unpack('!BI', packet[:5])
            body = packet[5:]
            print(f"→ robot_id={robot_id}, frame_size={frame_size}")

            if len(body) != frame_size:
                print(f"⚠️ 데이터 크기 불일치 (받음 {len(body)} vs 헤더 {frame_size})")
                continue

            # 이미지 디코딩
            img = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                print("❌ 이미지 디코딩 실패")
                continue

            # 화면 띄우기
            cv2.imshow(f"Robot{robot_id}", img)
            if cv2.waitKey(1) == 27:
                print("ESC 눌림, 종료")
                break

    except KeyboardInterrupt:
        print("Ctrl+C로 종료")

    except Exception as e:
        print(f"예외 발생: {e}")

    finally:
        sock.close()
        cv2.destroyAllWindows()
        print("종료 완료")

if __name__ == '__main__':
    main()
