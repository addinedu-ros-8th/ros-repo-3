import time
from ultralytics import YOLO
from server.ai_server.file_system_manager import FileSystemManager

CLASS_CODE = {
    "roscar": 0x00,
    "person": 0x01
}

class ObjectDetector:
    def __init__(self):
        self.model = YOLO(
            "https://github.com/addinedu-ros-8th/ros-repo-3/releases/download/v0.3.0/person+roscar_best.pt"
        )
        self.fs = FileSystemManager()
        self.last_result = None

    def on_frame_received(self, frame, roscar_id):
        """프레임 수신 시: 버퍼 저장 + 객체 감지 + 결과 반환 + 필요 시 클립 저장"""
        self.fs.add_frame(frame)
        self.last_result = self.run_detection(frame)

        # 감지된 객체가 유효할 경우만 저장
        if self.is_target_detected(self.last_result):
            trigger_time = time.time()
            self.fs.save_clip(trigger_time, roscar_id)

        return self.last_result

    def run_detection(self, frame):
        results = self.model(frame, conf=0.4, verbose=False)
        return results[0]

    def is_target_detected(self, results):
        """사람 또는 로봇 클래스가 감지되었는지 판단"""
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id]
            if cls_name in CLASS_CODE:
                return True
        return False
