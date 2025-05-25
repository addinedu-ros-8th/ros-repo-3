import struct

def handle_qrcode_search(main_service, req_data, client_socket):
    try:
        qr = req_data.get("qr_code")
        print(f"[IS] QR코드 조회 요청: {qr}")
        result = main_service.query.get_shoes_model_by_qrcode(qr)
        if result:
            cmd, status = b"IS", b"\x00"
            name = result.name.encode("utf-8")[:32].ljust(32, b"\x00")
            size = struct.pack("<I", result.size)
            color = result.color.value.encode("utf-8")[:16].ljust(16, b"\x00")
            quantity = struct.pack("<I", result.quantity)
            location = result.location.encode("utf-8")[:16].ljust(16, b"\x00")
            payload = cmd + status + name + size + color + quantity + location
        else:
            payload = b"IS\x01"
        client_socket.sendall(payload)
    except Exception as e:
        print(f"[❌] 상품 조회 실패: {e}")
        client_socket.sendall(b"IS\x01")