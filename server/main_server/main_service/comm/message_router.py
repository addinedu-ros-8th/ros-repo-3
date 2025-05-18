import struct

class MessageRouter:
    def __init__(self, main_service):
        self.main_service = main_service

    def route_message(self, message: bytes, client_socket):
        try:
            if len(message) < 2:
                print("[에러] 너무 짧은 메시지")
                return

            cmd = message[:2].decode('ascii', errors='replace')
            print(f"[CMD] {cmd}")

            if cmd == "AU":
                user_name = message[2:34].decode('utf-8').rstrip('\x00')
                password  = message[34:66].decode('utf-8').rstrip('\x00')
                payload = {"user_name": user_name, "password": password}
                self.main_service.handle_login_request(payload, client_socket)

            elif cmd == "IS":
                qr = message[2:].decode('utf-8').rstrip('\x00')
                payload = {"qr_code": qr}
                self.main_service.handle_qrcode_search(payload, client_socket)

            elif cmd == "IR":
                user_id    = struct.unpack(">I",  message[2:6])[0]
                item_count = struct.unpack(">H",  message[6:8])[0]
                destination= message[8:10].decode('utf-8')
                items = []
                offset = 10
                for _ in range(item_count):
                    sid, lid, qty = struct.unpack(">III", message[offset:offset+12])
                    items.append({
                        "shoes_model_id": sid,
                        "location_id":    lid,
                        "quantity":       qty
                    })
                    offset += 12
                payload = {
                    "user_id":    user_id,
                    "destination": destination,
                    "items":      items
                }
                self.main_service.handle_inventory_request(payload, client_socket)

            elif cmd == "CD":
                user_id, delivery_id = struct.unpack(">II", message[2:10])
                payload = {"user_id": user_id, "delivery_id": delivery_id}
                self.main_service.handle_cancel_task(payload, client_socket)

            elif cmd == "TR":
                user_id = struct.unpack(">I", message[2:6])[0]
                payload = {"user_id": user_id}
                self.main_service.handle_task_result_check(payload, client_socket)

            elif cmd == "IN":
                roscar_id = message[2]
                result_code = message[3]
                payload = {
                    "roscar_id": roscar_id,
                    "result_code": result_code
                }
                self.main_service.handle_ai_result(payload, client_socket)


            else:
                print(f"[❌ 알 수 없는 명령] {cmd}")
                client_socket.sendall(b"??")

        except Exception as e:
            print(f"[❌ route_message 에러] {e}")
            client_socket.sendall(b"!!")
