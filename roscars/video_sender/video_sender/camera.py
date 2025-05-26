import threading
import sys
import os
import re
import rclpy
from picamera2 import Picamera2
from .video_streamer import VideoStreamer
from .aruco_align    import ArucoAlign

def get_ssid_from_hostapd():
    """hostapd.conf에서 SSID 파싱 (AP 모드용)"""
    paths = [
        "/etc/hostapd/hostapd.conf",
        "/etc/hostapd.conf",
        "/etc/network/hostapd.conf",
        "/home/pinky/robot_ap.conf"
    ]
    for path in paths:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    for line in f:
                        m = re.match(r'^ssid=(.+)', line.strip())
                        if m:
                            return m.group(1).strip()
            except Exception:
                pass
    return "unknown"


def main():
    rclpy.init()
    align_node = ArucoAlign()
    spin_thread = threading.Thread(target=rclpy.spin, args=(align_node,), daemon=True)
    spin_thread.start()

    ssid = get_ssid_from_hostapd()

    cam = Picamera2()
    config = cam.create_preview_configuration(main={"size": (640, 480)})
    cam.configure(config)
    cam.start()

    streamer = VideoStreamer('192.168.4.18', 4436, quality=60)
    print(f"Streaming to 192.168.4.19:4436 with SSID={ssid}")

    try:
        while rclpy.ok():
            frame = cam.capture_array()
            # SSID 헤더와 함께 한 번에 전송
            streamer.send(frame.copy(), ssid)
            align_node.process_frame(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()