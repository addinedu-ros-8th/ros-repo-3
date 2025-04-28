# main.py

from ros_nodes.service_node import MainService
from logger import log_info, log_error

def main():
    try:
        log_info("[Main] Starting Main Service...")
        service = MainService()
        service.start()
    except KeyboardInterrupt:
        log_info("[Main] Main Service interrupted by user.")
    except Exception as e:
        log_error(f"[Main] Exception occurred: {str(e)}")

if __name__ == "__main__":
    main()
