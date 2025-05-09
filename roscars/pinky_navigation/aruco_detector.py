from pinkylib import Camera
import cv2
import numpy as np
import time

def get_transform_matrix(rvec, tvec):
    """Rodrigues 회전벡터와 이동벡터로 4x4 변환 행렬 생성"""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T

# 마커가 월드 원점에 고정되어 있다고 가정
T_world_marker = np.eye(4)

# 카메라가 로봇보다 z축 방향으로 6cm 위에 고정되어 있음
T_cam2roscar = np.eye(4)
T_cam2roscar[2, 3] = -0.06  # 단위: m

cam = Camera()
cam.set_calibration("camera_calibration.npz")
cam.start(width=640, height=480)

try:
    while True:
        frame = cam.get_frame()
        result_frame, pose_list = cam.detect_aruco(
            frame,
            aruco_dict_type=cv2.aruco.DICT_6X6_250,
            marker_size=0.02  # 마커 한 변의 실제 크기 (단위: m)
        )

        cam.display_jupyter(result_frame)

        if pose_list:
            for pose in pose_list:
                # pose가 리스트/튜플 형태라면 unpack
                if isinstance(pose, (list, tuple)) and len(pose) == 2:
                    rvec, tvec = pose
                else:
                    print("⚠️ 지원되지 않는 pose 형식:", pose)
                    continue

                # 변환 행렬 계산
                T_cam_marker = get_transform_matrix(rvec, tvec)
                T_marker_cam = np.linalg.inv(T_cam_marker)
                T_world_cam = T_world_marker @ T_marker_cam
                T_world_roscar = T_world_cam @ T_cam2roscar

                # 로봇 위치 출력
                position = T_world_roscar[:3, 3]
                print(f"🤖 로봇 위치 (x, y, z): {position.round(3)} m")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("🔌 인식 중단됨.")
finally:
    cam.close()
