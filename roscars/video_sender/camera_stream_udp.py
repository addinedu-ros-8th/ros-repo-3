import cv2
import platform

os_name = platform.system()
use_cv = False

def check_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "r") as f:
            return "Raspberry Pi" in f.read()
    except Exception:
        return False

def picamera2_init():
    from picamera2 import Picamera2

    picam2 = Picamera2()
    video_config = picam2.create_video_configuration()
    video_config.main.format = "RGB888"
    video_config.main.size = (640, 480)
    video_config.controls.FrameRate = 30
    picam2.configure(video_config)
    picam2.start()

if os_name == "Darwin": # MacOS
    cap = cv2.VideoCapture(1)
    use_cv = True
elif check_raspberry_pi():
    try:
        picamera2_init()
    except Exception as e:
        print("Picamera2 초기화 실패:", e)
else:  # Ubuntu 등 일반 Linux
    cap = cv2.VideoCapture(0)
    use_cv = True
