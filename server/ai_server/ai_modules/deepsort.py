import os
import time
from collections import defaultdict
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

class ObjectTracker:
    def __init__(self):
        self.tracker = DeepSort(max_age=30)
        self.track_times = defaultdict(dict)  # {roscar_id: {track_id: first_seen_time}}
        self.reported = defaultdict(set)      # {roscar_id: set(track_id)}

    def update(self, detections, frame, roscar_id):
        boxes = detections.boxes.xyxy.cpu().numpy() if detections.boxes.xyxy is not None else []
        confs = detections.boxes.conf.cpu().numpy() if detections.boxes.conf is not None else []
        clss = detections.boxes.cls.cpu().numpy().astype(int) if detections.boxes.cls is not None else []

        dets = []
        for box, conf, cls_id in zip(boxes, confs, clss):
            if cls_id == 0:  # Only track 'person' class
                x1, y1, x2, y2 = box
                dets.append(([x1, y1, x2 - x1, y2 - y1], conf, 'person'))

        tracks = self.tracker.update_tracks(dets, frame=frame)

        events = []
        now = time.time()
        for track in tracks:
            if not track.is_confirmed():
                continue
            track_id = track.track_id

            if track_id not in self.track_times[roscar_id]:
                self.track_times[roscar_id][track_id] = now
            elif track_id not in self.reported[roscar_id]:
                elapsed = now - self.track_times[roscar_id][track_id]
                if elapsed > 2.0:
                    self.reported[roscar_id].add(track_id)
                    events.append(track_id)

        return events
