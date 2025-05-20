import struct

class MessageRouter:
    def __init__(self, main_service):
        self.main_service = main_service
        self._handlers = {}
        self._notify_handlers = {}

    def register(self, cmd: str, handler):
        """
        일반 요청 핸들러 등록.
        handler signature: handler(main_service, payload: dict, client_socket)
        """
        self._handlers[cmd] = handler

    def register_notify(self, cmd: str, handler):
        """
        클라이언트 알림 핸들러 등록.
        handler signature: handler(main_service, client_socket, *args)
        """
        self._notify_handlers[cmd] = handler

    def notify(self, client_socket, cmd: str, *args):
        """
        등록된 알림 핸들러를 호출해 알림 전송.
        """
        h = self._notify_handlers.get(cmd)
        if not h:
            print(f"[notify] 알림 핸들러 없음: {cmd}")
            return
        h(self.main_service, client_socket, *args)

    def route_message(self, message: bytes, client_socket):
        try:
            # 타입 가드
            if isinstance(message, str):
                message = message.encode('utf-8')
            if not hasattr(client_socket, 'sendall'):
                print(f"[router] 잘못된 client_socket: {client_socket!r}")
                return

            print("=" * 60)
            print(f"[수신 바이트 HEX] {message.hex(' ').upper()}")
            print(f"[수신 길이] {len(message)} bytes")

            if len(message) < 2:
                print("[에러] 너무 짧은 메시지")
                client_socket.sendall(b"??")
                return

            cmd = message[:2].decode('utf-8', errors='ignore').strip()
            print(f"[CMD] {cmd}")

            handler = self._handlers.get(cmd)
            if not handler:
                print(f"[router] 알 수 없는 CMD: {cmd}")
                client_socket.sendall(b"??")
                return

            # 페이로드 파싱
            payload = {}
            if cmd == "AU":
                if len(message) < 66:
                    client_socket.sendall(b"AU\x01")
                    return
                payload["user_name"] = message[2:34].decode('utf-8').rstrip('\x00')
                payload["password"]  = message[34:66].decode('utf-8').rstrip('\x00')

            elif cmd == "IS":
                payload["qr_code"] = message[2:].decode('utf-8').rstrip('\x00')

            elif cmd == "IR":
                if len(message) < 10:
                    client_socket.sendall(b"IR\x01")
                    return
                user_id    = struct.unpack_from(">I", message, 2)[0]
                item_count = struct.unpack_from(">H", message, 6)[0]
                dest       = message[8:10].decode('utf-8').rstrip('\x00')
                offset = 10
                items = []
                for _ in range(item_count):
                    model = message[offset:offset+64].decode('utf-8').rstrip('\x00'); offset += 64
                    color = message[offset:offset+32].decode('utf-8').rstrip('\x00'); offset += 32
                    size  = struct.unpack_from(">I", message, offset)[0]; offset += 4
                    rack  = message[offset:offset+16].decode('utf-8').rstrip('\x00'); offset += 16
                    qty   = struct.unpack_from(">I", message, offset)[0]; offset += 4
                    items.append({"model":model, "color":color, "size":size, "rack":rack, "quantity":qty})
                mapped = []
                for it in items:
                    mid = self.main_service.query.get_model_id_by_name(it["model"])
                    lid = self.main_service.query.get_location_id_by_name(it["rack"])
                    if mid is None or lid is None: continue
                    mapped.append({"shoes_model_id":mid, "location_id":lid, "quantity":it["quantity"]})
                if not mapped:
                    client_socket.sendall(b"IR\x01")
                    return
                payload = {"user_id":user_id, "destination":dest, "items":mapped}

            elif cmd == "CD":
                if len(message) < 10:
                    client_socket.sendall(b"CD\x01")
                    return
                user_id, delivery_id = struct.unpack(">II", message[2:10])
                payload = {"user_id":user_id, "delivery_id":delivery_id}

            elif cmd == "TR":
                if len(message) < 6:
                    client_socket.sendall(b"TR\x01")
                    return
                user_id = struct.unpack(">I", message[2:6])[0]
                payload = {"user_id":user_id}

            elif cmd == "IN":
                ssid_len = message[2]
                ssid_end = 3 + ssid_len
                ssid     = message[3:ssid_end].decode('utf-8', errors='replace')
                result_code = message[ssid_end]
                angle_bytes = message[ssid_end+1:ssid_end+5]
                angle = struct.unpack(">f", angle_bytes)[0]
                roscar_id = self.main_service.query.get_roscar_id_by_name(ssid)
                if roscar_id is None:
                    client_socket.sendall(b"IN\x01")
                    return
                payload = {"roscar_id":roscar_id, "result_code":result_code, "angle":angle}

            # 핸들러 실행
            handler(self.main_service, payload, client_socket)

        except Exception as e:
            print(f"[route_message 예외] {e}")
            try:
                client_socket.sendall(b"!!")
            except Exception as inner:
                print(f"[‼ 예외 중 응답 실패] {inner}")