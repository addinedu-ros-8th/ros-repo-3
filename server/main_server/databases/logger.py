from server.main_server.databases.logger_function import LoggerFunction

class RoscarsLogWriter:
    def __init__(self, db):
        self.db = db
        self.logger_fn = LoggerFunction

    def _commit_log(self, log_obj):
        try:
            with self.db.get_session("roscars_log") as session:
                session.add(log_obj)
                session.commit()
        except Exception as e:
            print(f"[LogWriter] Logging failed: {e}")

    def log_roscar_event(self, roscar_id, task_id, event_type, camera_angle=None):
        log = self.logger_fn.build_roscar_event_log(roscar_id, task_id, event_type, camera_angle)
        self._commit_log(log)

    def log_task_event(self, task_id, previous, current):
        log = self.logger_fn.build_task_event_log(task_id, previous, current)
        self._commit_log(log)

    def log_delivery_event(self, delivery_id, user_id, previous, new):
        log = self.logger_fn.build_delivery_event_log(delivery_id, user_id, previous, new)
        self._commit_log(log)

    def log_delivery_event_failed(self, delivery_id, user_id):
        log = self.logger_fn.build_delivery_event_failed(delivery_id, user_id)
        self._commit_log(log)

    def log_task_event_failed(self, task_id):
        log = self.logger_fn.build_task_event_failed(task_id)
        self._commit_log(log)

    def log_precision_stop_result(self, roscar_id, task_id, is_success, deviation_cm):
        log = self.logger_fn.build_precision_stop_log(roscar_id, task_id, is_success, deviation_cm)
        self._commit_log(log)
