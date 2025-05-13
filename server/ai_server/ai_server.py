import socket
import cv2
import numpy as np
import struct
import zlib

MAX_UDP_SIZE = 65507

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 4436))
    print("수신 대기 중...")

    try:
        while True:
            packet, addr = sock.recvfrom(MAX_UDP_SIZE)
            if len(packet) < 5:
                continue
            robot_id, comp_len = struct.unpack('!BI', packet[:5])
            comp = packet[5:]
            if len(comp) != comp_len:
                continue

            # zlib 해제
            data = zlib.decompress(comp)
            img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue

            win = f"Robot_{robot_id}"
            cv2.imshow(win, img)
            if cv2.waitKey(1) == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
