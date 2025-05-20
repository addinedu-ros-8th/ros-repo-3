import struct

def handle_login_request(main_service, req_data, client_socket):
    try:
        print(f"[handle_login_request] user_name={req_data.get('user_name')}, password={req_data.get('password')}")
        user = main_service.query.get_user_by_name(req_data.get("user_name"))
        if user and user.check_password(req_data.get("password")):
            cmd = b"AU"
            status = b"\x00"
            user_id = struct.pack(">I", user.user_id)
            user_role = b"\x00" if user.user_role == "STAFF" else b"\x01"
            client_socket.sendall(cmd + status + user_id + user_role)
            print(f"[로그인 성공] user_id={user.user_id}, role={user.user_role}")
        else:
            client_socket.sendall(b"AU\x01")
            print("[❌ 로그인 실패] 사용자 정보 불일치")
    except Exception as e:
        print(f"[‼️ 예외 발생] {e}")
        try:
            client_socket.sendall(b"AU\x01")
        except:
            pass