import signal
import threading

class RuntimeController:
    def __init__(self):
        self.shutdown_flag = threading.Event()
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("[MAIN] 종료 요청 수신 (SIGINT)")
        self.shutdown_flag.set()

    def enable_auto_shutdown(self, seconds: int):
        def _shutdown():
            print(f"[TEST] {seconds}초 경과 → 종료 신호 발생")
            self.shutdown_flag.set()
        threading.Timer(seconds, _shutdown).start()