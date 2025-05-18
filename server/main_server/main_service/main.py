import signal
import threading
import struct
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

from server.main_server.main_service.comm.tcp_handler import TCPHandler
from server.main_server.main_service.comm.message_router import MessageRouter
from server.main_server.main_service.ros_interface.service.log_query_service import LogQueryService
from server.main_server.main_service.ros_interface.publisher.log_event_publisher import LogEventPublisher

from server.main_server.databases.database_manager import DatabaseManager
from server.main_server.databases.schema_manager import SchemaManager
from server.main_server.databases.logger import RoscarsLogWriter
from server.main_server.databases.query import MainServiceQuery
from server.main_server.databases.models.roscars_log_models import *
from server.main_server.databases.models.roscars_models import ShoesModel, RackLocation, Delivery, DestinationGroup, Task
from server.main_server.databases.utils import SensorUtils, MessageUtils

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

class MainService:
    def __init__(self, db: DatabaseManager):
        self.logger_session = db.get_session("roscars_log")
        self.logger = RoscarsLogWriter(self.logger_session)

        self.roscars_session = db.get_session("roscars")

        self.query = MainServiceQuery(
            roscars_session=self.roscars_session,
            roscars_log_session=self.logger_session
        )

        self.enable_shutdown_after_ai_result = False

    # [AU] 로그인 인증 요청
    def handle_login_request(self, req_data, client_socket):
        try:
            user_name = req_data.get("user_name")
            password = req_data.get("password")

            print(f"[handle_login_request] user_name={user_name}, password={password}")
            user = self.query.get_user_by_name(user_name)
            print(user)

            if user and user.check_password(password):
                # 응답 포맷: "AU"(2) + status(1) + user_id(4) + user_role(1)
                cmd = b"AU"
                status = bytes([0x00])
                user_id = struct.pack(">I", user.user_id)
                user_role = bytes([0x00 if user.user_role == "STAFF" else 0x01])

                response = cmd + status + user_id + user_role
                client_socket.sendall(response)
                print(f"[✅ 로그인 성공] user_id={user.user_id}, role={user.user_role}")
            else:
                # 실패 응답: "AU" + 0x01
                response = b"AU" + bytes([0x01])
                client_socket.sendall(response)
                print("[❌ 로그인 실패] 사용자 정보 불일치")

        except Exception as e:
            print(f"[‼️ 예외 발생] {e}")
            try:
                client_socket.sendall(b"AU" + bytes([0x01]))
            except:
                pass

    # [IS] 상품 정보 조회 요청 (QR코드 기반)
    def handle_qrcode_search(self, req_data, client_socket):
        try:
            qr_code_value = req_data.get("qr_code")
            print(f"[IS] QR코드 조회 요청: {qr_code_value}")
            result = self.query.get_shoes_model_by_qrcode(qr_code_value)

            if result:
                cmd = b"IS"
                status = struct.pack("B", 0x00)
                name = result.name.encode('utf-8')[:32].ljust(32, b'\x00')
                size = struct.pack("<I", result.size)
                color = result.color.value.encode('utf-8')[:16].ljust(16, b'\x00')
                quantity = struct.pack("<I", result.quantity)
                location = result.location.encode('utf-8')[:16].ljust(16, b'\x00')

                payload = cmd + status + name + size + color + quantity + location
                print(payload)
            else:
                payload = b"IS" + struct.pack("B", 0x01)

            client_socket.sendall(payload)

        except Exception as e:
            print("[❌] 상품 조회 실패:", e)
            client_socket.sendall(b"IS" + struct.pack("B", 0x01))

    def get_model_id_by_name(self, model_name: str) -> int | None:
        """
        Given a ShoesModel.name, return its shoes_model_id or None if not found.
        """
        return self.query.get_model_id_by_name(model_name)

    def get_location_id_by_name(self, rack_name: str) -> int | None:
        """
        Given a RackLocation.name, return its location_id or None if not found.
        """
        return self.query.get_location_id_by_name(rack_name)
    
    # [IR] 상품 요청
    def handle_inventory_request(self, req_data, client_socket):
        try:
            print("[IR] 상품 요청 시작")
            print(f"[IR] 수신 데이터: {req_data}")

            user_id     = req_data.get("user_id")
            destination = req_data.get("destination")  # ex) "G1"
            items       = req_data.get("items")

            # 필수값 검증
            if not user_id or not items or not isinstance(items, list) or not destination:
                print("[IR] 필수값 누락")
                # 실패: cmd(2B) + status(1B)
                body = b"IR" + struct.pack("B", 0x01)
                print(f"[IR] 실패 응답 → {body!r}")
                client_socket.sendall(body)
                return

            # 목적지 Enum 변환
            try:
                dest_enum = DestinationGroup[destination]
            except KeyError:
                print("[IR] 잘못된 목적지")
                body = b"IR" + struct.pack("B", 0x01)
                print(f"[IR] 실패 응답 → {body!r}")
                client_socket.sendall(body)
                return

            # Delivery 생성
            delivery = Delivery(
                user_id=user_id,
                roscar_id=None,
                delivery_status="TO_DO",
                driving_phase_id=None,
                destination=dest_enum
            )
            self.roscars_session.add(delivery)
            self.roscars_session.flush()

            # ✅ Delivery 로그 추가
            delivery_log = DeliveryEventLog(
                delivery_id=delivery.delivery_id,
                user_id=user_id,
                previous_event="WAIT",
                new_event="PROGRESS_START",
                timestamp=datetime.now()
            )
            self.logger_session.add(delivery_log)

            # Task 생성
            first_task_id = None
            for it in items:
                sid = it.get("shoes_model_id")
                lid = it.get("location_id")
                qty = it.get("quantity", 0)

                if not sid or not lid or qty <= 0:
                    continue

                # 유효성 체크
                if not self.roscars_session.get(ShoesModel, sid) \
                or not self.roscars_session.get(RackLocation, lid):
                    continue

                for _ in range(qty):
                    t = Task(
                        delivery_id=delivery.delivery_id,
                        shoes_model_id=sid,
                        location_id=lid,
                        #status="TO_DO"
                        status="IN_PROGRESS",
                    )
                    self.roscars_session.add(t)
                    self.roscars_session.flush()

                    # ✅ Task 로그 추가
                    task_log = TaskEventLog(
                        task_id=t.task_id,
                        previous_event="WAIT",
                        current_event="PROGRESS_START",
                        changed_at=datetime.now()
                    )
                    self.logger_session.add(task_log)

                    if first_task_id is None:
                        first_task_id = t.task_id

            self.logger_session.commit()

            # 성공/실패에 따라 body 생성
            if not first_task_id:
                # 실패
                body = b"IR" + struct.pack("B", 0x01)
            else:
                # 성공
                body = (
                    b"IR" +
                    struct.pack("B", 0x00) +
                    struct.pack(">I", delivery.delivery_id) +
                    struct.pack(">I", first_task_id)
                )

            # unpack 버전 로깅
            if len(body) >= 11:
                cmd_bytes, status, did, tid = struct.unpack(">2sBII", body)
                cmd = cmd_bytes.decode('ascii')
                print(f"[IR] 응답 → cmd={cmd}, status={status}, delivery_id={did}, first_task_id={tid}")
            else:
                cmd_bytes, status = struct.unpack(">2sB", body)
                cmd = cmd_bytes.decode('ascii')
                print(f"[IR] 실패 응답 → cmd={cmd}, status={status}")

            client_socket.sendall(body)

        except Exception as e:
            print(f"[❌ IR 예외]: {e}")
            # 예외 시에도 최소 cmd+status
            fallback = b"IR" + struct.pack("B", 0x01)
            print(f"[IR] 예외 실패 응답 → {fallback!r}")
            client_socket.sendall(fallback)

    
    # [CD] 배송 취소
    def handle_cancel_task(self, req_data, client_socket):
        try:
            user_id = req_data.get("user_id")
            delivery_id = req_data.get("delivery_id")

            if not user_id or not delivery_id:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 1. Delivery 및 연결된 Task 조회
            delivery = self.roscars_session.query(Delivery).filter_by(delivery_id=delivery_id).first()

            if not delivery:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            tasks = delivery.tasks
            if not tasks:
                response = { "cmd": "CD", "status": 0x01 }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 2. 첫 번째 Task 기준 상태 확인
            first_task = sorted(tasks, key=lambda t: t.task_id)[0]

            if first_task.status == "DONE":
                # 취소 불가
                response = { "cmd": "CD", "status": 0x01, "delivery_id": delivery_id }
                client_socket.sendall(MessageUtils.success(response).encode("utf-8"))
                return

            # 3. 상태 변경
            delivery.delivery_status = "CANCELLED"
            self.roscars_session.commit()

            # TODO: 4. ROS2 메시지 전송
            # self.ros2_publisher.send_cancel_command(delivery_id=delivery_id)

            response = { "cmd": "CK", "status": 0x00, "delivery_id": delivery_id }

            self.logger.log_delivery_event(
            delivery_id=delivery.delivery_id,
            user_id=user_id,
            previous=DefaultEventType.PROGRESS_START,
            new=DefaultEventType.CANCEL
            )


        except Exception as e:
            self.roscars_session.rollback()
            try:
                self.logger.log_delivery_event(
                    delivery_id=delivery_id,
                    user_id=user_id,
                    previous=DefaultEventType.PROGRESS_START,
                    new=DefaultEventType.FAILE
                )
            except:
                pass
            response = { "cmd": "CK", "status": 0x01, "delivery_id": delivery_id }


        client_socket.sendall(MessageUtils.success(response).encode("utf-8"))


    # [TR] 작업 결과 확인 
    def handle_task_result_check(self, req_data, client_socket):
        print("[handle_task_result_check] 함수 시작, req_data=", req_data)
        try:
            user_id = req_data.get("user_id")
            print(f"[handle_task_result_check] 추출된 user_id: {user_id}")

            # user_id 누락 시 실패 응답 처리
            if not user_id:
                print("[handle_task_result_check] 사용자 ID 누락, 실패 응답 처리")
                body = b"TR" + struct.pack(">B", 0x01)
                print(f"[handle_task_result_check] 전송 바디(실패): {body.hex(' ').upper()}")
                client_socket.sendall(body)
                return

            # Delivery 조회
            deliveries = self.roscars_session.query(Delivery).filter_by(user_id=user_id).all()
            print(f"[handle_task_result_check] 조회된 Delivery 개수: {len(deliveries)}")

            # 조회 결과 없으면 기본 응답(0,0)
            if not deliveries:
                print("[handle_task_result_check] deliveries 없음, 기본 응답(0,0)")
                body = b"TR" + struct.pack(">BII", 0x00, 0, 0)
                print(f"[handle_task_result_check] 전송 바디(0개): {body.hex(' ').upper()}")
                client_socket.sendall(body)
                return

            # 완료/진행 중 카운트 계산
            done_count = 0
            in_progress_names = set()
            for delivery in deliveries:
                for task in delivery.tasks:
                    if task.status == "DONE":
                        done_count += 1
                    elif task.status == "IN_PROGRESS":
                        in_progress_names.add(task.shoes_model.name)
            in_progress_count = len(in_progress_names)
            print(f"[handle_task_result_check] done_count={done_count}, in_progress_count={in_progress_count}")

            # 성공 응답 생성 및 전송
            body = b"TR" + struct.pack(">BII", 0x00, done_count, in_progress_count)
            print(f"[handle_task_result_check] 전송 바디(성공): {body.hex(' ').upper()}")
            client_socket.sendall(body)

        except Exception as e:
            print(f"[handle_task_result_check] 예외 발생: {e}")
            body = b"TR" + struct.pack(">B", 0x01)
            print(f"[handle_task_result_check] 전송 바디(예외 실패): {body.hex(' ').upper()}")
            client_socket.sendall(body)

    # [TR_NOTIFY] 작업 상태 변경 알림
    def send_task_status_notification(self, client_socket, task_id: int, delivery_id: int, status: str, shoes_model_name: str):
        try:
            print(f"[TR_NOTIFY] 상태 알림 → task_id={task_id}, status={status}, model={shoes_model_name}")

            cmd = b"TR"
            delivery_bytes = struct.pack(">I", delivery_id)
            task_bytes     = struct.pack(">I", task_id)

            status_map = {
                "TO_DO": 0x00,
                "IN_PROGRESS": 0x01,
                "DONE": 0x02,
                "CANCELLED": 0x03
            }
            status_code = status_map.get(status.upper(), 0xFF)
            status_bytes = struct.pack("B", status_code)

            name_bytes = shoes_model_name.encode('utf-8')[:32].ljust(32, b'\x00')
            timestamp_ms = int(time.time() * 1000)
            timestamp_bytes = struct.pack(">Q", timestamp_ms)

            payload = cmd + delivery_bytes + task_bytes + status_bytes + name_bytes + timestamp_bytes
            client_socket.sendall(payload)

            print(f"[TR_NOTIFY] 전송 완료")

        except Exception as e:
            print(f"[❌ TR_NOTIFY 전송 실패]: {e}")

    # [IN] AI 인식 결과 수신
    def handle_ai_result(self, req_data, client_socket):
        response = { "cmd": "IN", "status": 0x01 }

        try:
            roscar_id = req_data.get("roscar_id")
            result_code = req_data.get("result_code")

            if roscar_id is None or result_code not in [0x00, 0x01]:
                print(f"[IN] 잘못된 요청: roscar_id={roscar_id}, result_code={result_code}")
            else:
                detected_object = {
                    0x00: "Roscar",
                    0x01: "Person",
                }.get(result_code, "Unknown")

                print(f"[IN] 객체 인식 결과 수신 → ID={roscar_id}, code={result_code} ({detected_object})")

                # 로그 기록
                self.logger.log_roscar_event(
                    roscar_id=roscar_id,
                    task_id=None,
                    event_type=RosCarEventType.OBJECT_DETECTED
                )

                # ROS2 전송 예시 (주석 해제 시 사용)
                # self.ros2_publisher.send_emergency_stop(roscar_id)

                response["status"] = 0x00

            if self.enable_shutdown_after_ai_result:
                print("[AI-TEST] IN 수신 확인 → 자동 종료 트리거")
                self.shutdown_flag.set()


        except Exception as e:
            print(f"[IN] 예외 발생: {e}")
            try:
                self.logger.log_roscar_event(
                    roscar_id=roscar_id if 'roscar_id' in locals() else -1,
                    task_id=None,
                    event_type=RosCarEventType.OBJECT_DETECTED
                )
            except:
                pass

        finally:
            client_socket.sendall(MessageUtils.success(response, "IN").encode("utf-8"))
    
    def register_shutdown_flag(self, shutdown_flag):
        self.shutdown_flag = shutdown_flag


# ==========================
# 실행 엔트리포인트
# ==========================
def main(main_test_mode=False, ai_test_mode=False):
    runtime = RuntimeController()  

    # DB 초기화
    db = DatabaseManager()
    schema = SchemaManager(db)
    schema.check_db_init()
    print("[MAIN] DB 연결 및 구조 확인 완료")

    # 서비스 및 라우터 준비
    main_service = MainService(db)
    main_service.register_shutdown_flag(runtime.shutdown_flag)

    router = MessageRouter(main_service)

    if ai_test_mode:
        main_service.enable_shutdown_after_ai_result = True

    # TCP 서버 실행
    port = 5001 if ai_test_mode else 9000
    tcp_server = TCPHandler("0.0.0.0", port, router)
    tcp_thread = threading.Thread(target=tcp_server.start_server, daemon=True)
    tcp_thread.start()
    print(f"[MAIN] TCP 서버 시작 (port={port})")

    # ROS2 초기화 및 노드 실행
    rclpy.init()
    logger = RoscarsLogWriter(db.get_session("roscars_log"))

    # 노드 인스턴스 생성
    sensor_node = SensorUtils(logger, db)
    log_query_node = LogQueryService()
    log_event_publisher = LogEventPublisher()

    # 멀티스레드 실행기 생성 및 노드 등록
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(log_query_node)
    executor.add_node(log_event_publisher)

    if main_test_mode:
        runtime.enable_auto_shutdown(3)

    try:
        while not runtime.shutdown_flag.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    except rclpy.executors.ExternalShutdownException:
        print("[MAIN] ROS2가 종료되어 spin 종료됨")

    finally:
        print("[MAIN] 종료 처리 중...")
        sensor_node.destroy_node()
        log_query_node.destroy_node()
        log_event_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        tcp_server.shutdown_server()
        tcp_thread.join(timeout=2)
        print("[MAIN] 종료 완료")


if __name__ == '__main__':
    import sys
    main(
        main_test_mode="--test" in sys.argv,
        ai_test_mode="--ai-test" in sys.argv
    )