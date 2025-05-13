# ai_server_receiver.py
import socket
import cv2
import numpy as np

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 4436))
    print("AI 서버 UDP 수신 대기 중 (포트 4436)...")

    try:
        while True:
            data, addr = sock.recvfrom(65536)
            print(f"수신됨: {len(data)} bytes from {addr}")

            if not data:
                print("빈 데이터 수신됨")
                continue

            img_array = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if img is not None:
                cv2.imshow("UDP Image", img)
                if cv2.waitKey(1) == 27:  # ESC key
                    print("ESC 눌림, 종료합니다.")
                    break
            else:
                print("이미지 디코딩 실패")

    except KeyboardInterrupt:
        print("Ctrl+C 종료")

    except Exception as e:
        print(f"예외 발생: {e}")

    finally:
        sock.close()
        cv2.destroyAllWindows()
        print("종료 완료")

if __name__ == '__main__':
    main()
