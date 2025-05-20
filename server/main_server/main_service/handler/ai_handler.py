import time
import struct
from server.main_server.databases.models.roscars_log_models import RosCarEventType

def handle_ai_result(main_service, req_data, client_socket):
    status = 0x01
    try:
        rid, code, angle = req_data.get("roscar_id"), req_data.get("result_code"), req_data.get("angle")
        etype = {0x00: "OBJECT_DETECTED_ROSCAR", 0x01: "OBJECT_DETECTED_PERSON"}.get(code)
        event = getattr(RosCarEventType, etype)
        main_service.logger.log_roscar_event(roscar_id=rid, task_id=None,
                                            event_type=event,
                                            camera_angle=int(angle) if angle else None)
        status = 0x00
        if main_service.enable_shutdown_after_ai_result:
            main_service.shutdown_flag.set()
    except Exception as e:
        print(f"[IN] 예외: {e}")
    finally:
        client_socket.sendall(b"IN" + bytes([status]))