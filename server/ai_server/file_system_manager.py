from collections import deque
import time
import cv2
import os

class FileSystemManager:
    def __init__(self, buffer_duration=7.0, output_dir="recordings"):
        self.buffer = deque()
        self.buffer_duration = buffer_duration
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def add_frame(self, frame):
        ts = time.time()
        self.buffer.append((ts, frame))
        self._trim_buffer()

    def _trim_buffer(self):
        cutoff = time.time() - self.buffer_duration
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()

    def save_clip(self, trigger_time, roscar_id):
        clip = [
            (ts, frame) for ts, frame in self.buffer
            if trigger_time - self.buffer_duration <= ts <= trigger_time + self.buffer_duration
        ]
        if not clip:
            print("No frames to save.")
            return

        filename = f"{self.output_dir}/roscar_{roscar_id}_{int(trigger_time)}.avi"
        height, width, _ = clip[0][1].shape
        writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"XVID"), 10, (width, height))

        for _, frame in clip:
            writer.write(frame)
        writer.release()
        print(f"[FileSystem] Saved: {filename}")
