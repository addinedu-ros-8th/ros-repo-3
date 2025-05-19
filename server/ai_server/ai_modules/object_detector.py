import os
import time
import requests
from ultralytics import YOLO
from server.ai_server.file_system_manager import FileSystemManager
from server.ai_server.ai_modules.deepsort import ObjectTracker

CLASS_CODE = {
    "Roscar": 0x00,
    "Person": 0x01
}

class ObjectDetector:
    def __init__(self):
        self.fs = FileSystemManager()
        self.last_result = None
        self.tracker = ObjectTracker()

        model_url = "https://github.com/addinedu-ros-8th/ros-repo-3/releases/download/v0.3.0/person+roscar_best.pt"
        model_dir = "server/ai_server/ai_modules"
        model_path = self.download_model(model_url, model_dir)
        self.model = YOLO(model_path)

    def download_model(self, url: str, save_dir: str) -> str:
        os.makedirs(save_dir, exist_ok=True)
        filename = os.path.basename(url)
        save_path = os.path.join(save_dir, filename)

        if not os.path.exists(save_path):
            print(f"[AI] Downloading model to {save_path}...")
            response = requests.get(url, stream=True)
            with open(save_path, "wb") as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            print("[AI] Download completed.")
        else:
            print(f"Model already exists at {save_path}")

        return save_path

    def on_frame_received(self, frame, roscar_id):
        self.fs.add_frame(frame)
        self.last_result = self.run_detection(frame)

        # 🔍 인식된 클래스 이름만 출력
        detected_classes = set()
        for box in self.last_result.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.last_result.names[cls_id]
            detected_classes.add(cls_name)

        if detected_classes:
            print(f"[DETECTED] {', '.join(sorted(detected_classes))}")

        # ✅ DeepSORT 기반 추적자에게 현재 결과 전달
        tracked_persons = self.tracker.update(self.last_result, frame, roscar_id)
        if tracked_persons:
            print(f"[✅] 2초 이상 감지된 person ID: {tracked_persons}")

        # 🎥 감지된 객체 있으면 클립 저장
        if self.is_target_detected(self.last_result):
            trigger_time = time.time()
            self.fs.save_clip(trigger_time, roscar_id)

        return self.last_result

    def run_detection(self, frame):
        results = self.model(frame, conf=0.4, verbose=False)
        return results[0]

    def is_target_detected(self, results):
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id]
            if cls_name in CLASS_CODE:
                return True
        return False