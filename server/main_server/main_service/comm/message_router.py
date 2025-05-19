import struct

class MessageRouter:
    def __init__(self, main_service):
        self.main_service = main_service

    def route_message(self, message: bytes, client_socket):
        try:
            print("=" * 60)
            print(f"[📥 수신 바이트 HEX] {message.hex(' ').upper()}")
            print(f"[📥 수신 길이] {len(message)} bytes")

            if len(message) < 2:
                print("[❌ 에러] 너무 짧은 메시지 (2바이트 미만)")
                client_socket.sendall(b"??")
                return

            cmd = message[:2].decode('utf-8', errors='ignore').strip()
            print(f"[🔍 CMD] {cmd}")

            if cmd == "AU":
                print("[👤 로그인 요청 처리 시작]")
                if len(message) < 66:
                    print(f"[❌ AU] 메시지 길이 부족 (받은 {len(message)} / 필요 66)")
                    client_socket.sendall(b"AU\x01")
                    return

                user_name = message[2:34].decode('utf-8').rstrip('\x00')
                password  = message[34:66].decode('utf-8').rstrip('\x00')
                print(f"[🆔 사용자 ID] {user_name}")
                print(f"[🔑 비밀번호] {password}")
                payload = {"user_name": user_name, "password": password}
                self.main_service.handle_login_request(payload, client_socket)

            elif cmd == "IS":
                print("[🔎 QR코드 상품 조회 요청 처리 시작]")
                qr = message[2:].decode('utf-8').rstrip('\x00')
                print(f"[📦 QR코드] {qr}")
                payload = {"qr_code": qr}
                self.main_service.handle_qrcode_search(payload, client_socket)

            elif cmd == "IR":
                print("[📦 인벤토리 요청 처리 시작]")
                if len(message) < 10:
                    print(f"[❌ IR] 헤더 부족 (받은 {len(message)} / 최소 10)")
                    client_socket.sendall(b"IR\x01")
                    return

                user_id = struct.unpack_from(">I", message, 2)[0]
                item_count = struct.unpack_from(">H", message, 6)[0]
                destination = message[8:10].decode('utf-8').rstrip('\x00')
                print(f"[👤 사용자 ID] {user_id}")
                print(f"[🧾 요청 아이템 수] {item_count}")
                print(f"[📍 목적지] {destination}")

                expected_len = 10 + item_count * (64 + 32 + 4 + 16 + 4)
                if len(message) < expected_len:
                    print(f"[❌ IR] 메시지 길이 부족 (받은 {len(message)} / 필요 {expected_len})")
                    client_socket.sendall(b"IR\x01")
                    return

                items = []
                offset = 10
                for i in range(item_count):
                    model = message[offset:offset+64].decode('utf-8').rstrip('\x00'); offset += 64
                    color = message[offset:offset+32].decode('utf-8').rstrip('\x00'); offset += 32
                    size = struct.unpack_from(">I", message, offset)[0]; offset += 4
                    rack = message[offset:offset+16].decode('utf-8').rstrip('\x00'); offset += 16
                    quantity = struct.unpack_from(">I", message, offset)[0]; offset += 4

                    print(f"[🧩 아이템 {i+1}] model={model}, color={color}, size={size}, rack={rack}, quantity={quantity}")
                    items.append({
                        "model": model,
                        "color": color,
                        "size": size,
                        "rack": rack,
                        "quantity": quantity
                    })

                mapped = []
                for it in items:
                    mid = self.main_service.get_model_id_by_name(it["model"])
                    lid = self.main_service.get_location_id_by_name(it["rack"])

                    print(f"[🔗 매핑] model_id={mid}, location_id={lid}")
                    if mid is None or lid is None:
                        print(f"[❌ 매핑 실패] model={it['model']}, rack={it['rack']}")
                        continue

                    mapped.append({
                        "shoes_model_id": mid,
                        "location_id":    lid,
                        "quantity":       it["quantity"]
                    })

                if not mapped:
                    print("[❌ IR] 유효한 아이템이 없음 (매핑 실패)")
                    client_socket.sendall(b"IR\x01")
                    return

                payload = {
                    "user_id": user_id,
                    "destination": destination,
                    "items": mapped
                }
                self.main_service.handle_delivery_request(payload, client_socket)

            elif cmd == "CD":
                print("[🚫 배송 취소 요청 처리 시작]")
                if len(message) < 10:
                    print(f"[❌ CD] 길이 부족 (받은 {len(message)} / 필요 10)")
                    client_socket.sendall(b"CD\x01")
                    return

                user_id, delivery_id = struct.unpack(">II", message[2:10])
                print(f"[👤 사용자 ID] {user_id}, [📦 배송 ID] {delivery_id}")
                payload = {"user_id": user_id, "delivery_id": delivery_id}
                self.main_service.handle_cancel_task(payload, client_socket)

            elif cmd == "TR":
                print("[📊 작업 결과 확인 요청 처리 시작]")
                if len(message) < 6:
                    print(f"[❌ TR] 길이 부족 (받은 {len(message)} / 필요 6)")
                    client_socket.sendall(b"TR\x01")
                    return

                user_id = struct.unpack(">I", message[2:6])[0]
                print(f"[👤 사용자 ID] {user_id}")
                payload = {"user_id": user_id}
                self.main_service.handle_task_result_check(payload, client_socket)

            elif cmd == "IN":
                ssid_len = message[2]
                if len(message) < 3 + ssid_len + 1:
                    print(f"[❌ IN] 메시지 길이 부족 (받은 {len(message)} / 필요 {3 + ssid_len + 1})")
                    client_socket.sendall(b"IN\x01")
                    return

                ssid_end = 3 + ssid_len
                ssid = message[3:ssid_end].decode('utf-8', errors='replace')
                result_code = message[ssid_end]

                print(f"[IN] 결과 수신: SSID={ssid}, 코드={result_code:#04x}")

                # ✅ Roscar ID 조회 (roscar_name = ssid)
                roscar_id = self.main_service.query.get_roscar_id_by_name(ssid)
                if roscar_id is None:
                    print(f"[❌ IN] SSID({ssid})에 해당하는 roscar_id 없음")
                    client_socket.sendall(b"IN\x01")
                    return

                payload = {
                    "roscar_id": roscar_id,
                    "result_code": result_code
                }
                self.main_service.handle_ai_result(payload, client_socket)

        except Exception as e:
            print(f"[❌ route_message 예외] {e}")
            try:
                client_socket.sendall(b"!!")
            except Exception as inner_e:
                print(f"[‼️ 예외 중 응답 실패] {inner_e}")
