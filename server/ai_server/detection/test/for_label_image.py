import socket
import zlib
import cv2
import numpy as np
import os

PORT = 9999

# 사용자 다운로드 폴더 기준 설정
home_dir = os.path.expanduser('~')
base_save_dir = os.path.join(home_dir, 'Downloads', 'saved_frames')

# 거리별 폴더 설정
distance_categories = ['0_50', '50_100', '100_150', '150_200', '200_250', '250_300']

# 모든 거리 폴더 생성
for category in distance_categories:
    os.makedirs(os.path.join(base_save_dir, category), exist_ok=True)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', PORT))

print("로컬 영상 수신 대기 중...")

frame_count = 0  # 수신된 총 프레임 수
save_count = {category: 0 for category in distance_categories}  # 각 폴더별 저장 개수

current_category = '0_50'  # 초기 거리 설정
recording = False           # 녹화 여부 플래그
recorded_images = 0         # 현재 녹화 세션 동안 저장한 이미지 수

print("➡️ 현재 거리 범주:", current_category)
print("키 입력 안내: 1=0_50 / 2=50_100 / 3=100_150 / 4=150_200 / 5=200_250 / 6=250_300 / s=녹화 시작 / e=녹화 중단 / ESC=종료")

while True:
    data, _ = sock.recvfrom(65536)  # 64KB 제한
    try:
        decompressed = zlib.decompress(data)
        np_data = np.frombuffer(decompressed, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow('Live from Pi Camera', frame)

            frame_count += 1

            # 녹화 중일 때만 저장
            if recording and frame_count % 5 == 0:
                filename = os.path.join(base_save_dir, current_category, f"{current_category}_{save_count[current_category]:05d}.jpg")
                cv2.imwrite(filename, frame)
                save_count[current_category] += 1
                recorded_images += 1
                print(f"✅ 저장 완료: {filename} (이번 세션 {recorded_images}/100)")

                # 100장 저장되면 녹화 중단
                if recorded_images >= 100:
                    recording = False
                    recorded_images = 0
                    print("⏹️ 자동 녹화 중단 (100장 저장 완료)")

            key = cv2.waitKey(1) & 0xFF

            if key == 27:  # ESC 종료
                break
            elif key == ord('1'):
                current_category = '0_50'
                print("🔄 거리 변경: 0_50")
            elif key == ord('2'):
                current_category = '50_100'
                print("🔄 거리 변경: 50_100")
            elif key == ord('3'):
                current_category = '100_150'
                print("🔄 거리 변경: 100_150")
            elif key == ord('4'):
                current_category = '150_200'
                print("🔄 거리 변경: 150_200")
            elif key == ord('5'):
                current_category = '200_250'
                print("🔄 거리 변경: 200_250")
            elif key == ord('6'):
                current_category = '250_300'
                print("🔄 거리 변경: 250_300")
            elif key == ord('s'):
                recording = True
                
                recorded_images = 0  # 새 세션 시작할 때 리셋
                print("⏺️ 녹화 시작")
            elif key == ord('e'):
                recording = False
                recorded_images = 0
                print("⏹️ 수동 녹화 중단")

    except Exception as e:
        print("❌ 프레임 처리 에러:", e)
        continue

sock.close()
cv2.destroyAllWindows()
