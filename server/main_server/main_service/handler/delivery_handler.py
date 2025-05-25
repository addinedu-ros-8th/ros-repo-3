import struct
from datetime import datetime
from server.main_server.databases.models.roscars_log_models import DeliveryEventLog, TaskEventLog
from server.main_server.databases.models.roscars_models import Delivery, Task, OperationalStatus, RosCars, DestinationGroup


def handle_delivery_request(main_service, req_data, client_socket):
    try:
        print(f"[IR] 상품 요청 시작, 데이터={req_data}")
        user_id = req_data.get("user_id")
        dest = req_data.get("destination")
        items = req_data.get("items")
        if not user_id or not dest or not isinstance(items, list):
            raise ValueError("필수값 누락")

        try:
            dest_enum = DestinationGroup[dest]
        except KeyError:
            raise ValueError("잘못된 목적지")

        roscar = (
            main_service.roscars_session
            .query(RosCars)
            .filter_by(operational_status=OperationalStatus.STANDBY)
            .order_by(RosCars.battery_percentage.desc())
            .first()
        )
        if not roscar:
            raise RuntimeError("사용 가능한 Roscar 없음")

        delivery = Delivery(
            user_id=user_id,
            roscar_id=roscar.roscar_id,
            delivery_status="TO_DO",
            driving_phase_id=None,
            destination=dest_enum
        )
        main_service.roscars_session.add(delivery)
        main_service.roscars_session.flush()

        # 초기 Delivery 이벤트 로그
        main_service.logger_session.add(
            DeliveryEventLog(
                delivery_id=delivery.delivery_id,
                user_id=user_id,
                previous_event="WAIT",
                new_event="PROGRESS_START",
                timestamp=datetime.now()
            )
        )

        first_task = None
        # 각 아이템 딕셔너리당 하나의 Task 생성
        for it in items:
            sid = it.get("shoes_model_id")
            lid = it.get("location_id")

            t = Task(
                delivery_id=delivery.delivery_id,
                shoes_model_id=sid,
                location_id=lid,
                status="TO_DO"
            )
            main_service.roscars_session.add(t)
            main_service.roscars_session.flush()

            main_service.logger_session.add(
                TaskEventLog(
                    task_id=t.task_id,
                    previous_event="WAIT",
                    current_event="PROGRESS_START",
                    changed_at=datetime.now()
                )
            )
            first_task = first_task or t.task_id

        # 세션 커밋: Delivery, Task 및 로그
        main_service.roscars_session.commit()
        main_service.logger_session.commit()

        # 배송 시작 클라이언트 호출
        main_service._launch_start_delivery_client(roscar, delivery)

        if not first_task:
            raise RuntimeError("Task 생성 실패")

        # 응답 페이로드 작성
        body = b"IR" + b"\x00" + struct.pack(">I", delivery.delivery_id) + struct.pack(">I", first_task)
        client_socket.sendall(body)
        print(f"[IR] 성공 응답: {body.hex().upper()}")

    except Exception as e:
        print(f"[IR] 오류: {e}")
        client_socket.sendall(b"IR\x01")


def handle_cancel_task(main_service, req_data, client_socket):
    delivery_id = req_data.get("delivery_id") or 0
    user_id = req_data.get("user_id")
    try:
        if not delivery_id or not user_id:
            raise ValueError
        delivery = main_service.roscars_session.query(Delivery).get(delivery_id)
        if not delivery:
            raise RuntimeError
        delivery.delivery_status = "DONE"
        main_service.roscars_session.commit()
        client_socket.sendall(b"CD\x00" + struct.pack(">I", delivery_id))
    except:
        client_socket.sendall(b"CD\x01" + struct.pack(">I", delivery_id))
