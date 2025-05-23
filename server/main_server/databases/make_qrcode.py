import qrcode
import os

# QR 코드 값 리스트
qr_values = [
    "Puma Palermo Trainers | 240 | HYPERLINK_BLUE_FLAME_FLICKER_GUM | R3-G4-S1 | 5",
    "Nike Air Rift | 240 | BLACK | R3-G2-S2 | 3",
    "Adidas Gazelle | 260 | HOTPINK | R3-G2-S2 | 7"
]

# 현재 작업 디렉토리 기준 쓰기 가능한 위치 지정
output_dir = os.path.join(os.getcwd(), "qr_codes")
os.makedirs(output_dir, exist_ok=True)

for i, value in enumerate(qr_values, start=1):
    img = qrcode.make(value)
    filename = f"qr_{i}.png"
    path = os.path.join(output_dir, filename)
    img.save(path)
    print(f"Saved {path}")
