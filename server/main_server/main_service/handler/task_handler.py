import struct
from server.main_server.databases.models.roscars_models import Task, Delivery, TaskStatus

def handle_task_result_check(main_service, req_data, client_socket):
    print("[handle_task_result_check] req=", req_data)
    user_id = req_data.get("user_id")
    if not user_id:
        client_socket.sendall(b"TR" + struct.pack(">BHB", 0x01, 0, 0))
        return
    done = main_service.roscars_session.query(Task)\
        .join(Delivery)\
        .filter(Delivery.user_id==user_id, Task.status==TaskStatus.DONE)\
        .count()
    inprog_tasks = main_service.roscars_session.query(Task)\
        .join(Delivery)\
        .filter(Delivery.user_id==user_id,
                Task.status==TaskStatus.IN_PROGRESS,
                Delivery.delivery_status=="IN_PROGRESS")\
        .all()
    inprog_map = {}
    for t in inprog_tasks:
        inprog_map.setdefault(t.delivery_id, t.shoes_model.name)
    cnt = len(inprog_map)
    header = struct.pack(">BHB", 0x00, done, cnt)
    body = b"TR" + header
    for did, name in inprog_map.items():
        body += struct.pack(">I", did)
        body += name.encode("utf-8")[:32].ljust(32, b"\x00")
    client_socket.sendall(body)

def send_task_status_notification(main_service, client_socket, task_id, delivery_id, status, name):
    # 알림 전송 로직
    pass