uint32 event_id
uint8 event_type       # 0x00=EMERGENCY_STOP, 0x01=DELIVERY_START 등
string event_data      # JSON 직렬화된 상세 내용
builtin_interfaces/Time stamp
