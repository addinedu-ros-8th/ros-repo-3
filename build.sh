#!/bin/bash

set -e

# 시스템 패키지 최신화
echo "[INFO] 시스템 패키지 최신화 중..."
sudo apt update && sudo apt upgrade -y

# [GUI] pyzbar 의존성 설치
sudo apt install libzbar0 libxcb-cursor0 -y

# ROS2
echo "[INFO] 시스템 의존성 설치 중..."
sudo apt install -y \
    ros-jazzy-domain-bridge \
    ros-jazzy-tf-transformations \
    libgpiod-dev

# 가상환경 설정
VENV_DIR=".roscars_venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] 가상환경 생성: $VENV_DIR"
    python3.12 -m venv "$VENV_DIR"
fi

echo "[INFO] 가상환경 활성화"
source "$VENV_DIR/bin/activate"

# Python 패키지 설치
echo "[INFO] Python 의존성 설치 중..."
pip install -r requirements.txt

# ROS2 설정 적용
echo "[INFO] ROS2 환경 설정 적용 중..."
source /opt/ros/jazzy/setup.bash

# 빌드 캐시 및 __pycache__ 정리
echo "[INFO] 이전 빌드 캐시 정리..."
rm -rf build/ install/ log/
export PYTHONWARNINGS="ignore::UserWarning"
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

# 환경 변수에서 유효하지 않은 경로 제거
echo "[INFO] AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH 정리..."

filter_existing_paths() {
    local input="$1"
    local result=""
    IFS=':' read -ra paths <<< "$input"
    for path in "${paths[@]}"; do
        if [ -d "$path" ]; then
            result+="${path}:"
        fi
    done
    echo "${result%:}"
}

export AMENT_PREFIX_PATH=$(filter_existing_paths "$AMENT_PREFIX_PATH")
export CMAKE_PREFIX_PATH=$(filter_existing_paths "$CMAKE_PREFIX_PATH")
echo "[INFO] AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
echo "[INFO] CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

# colcon 빌드
echo "[INFO] colcon 빌드 시작..."
colcon build \
  --cmake-args \
    "-DCMAKE_CXX_FLAGS=-Wno-error=unused-parameter -Wno-error=pedantic -Wno-error=sign-compare" \
  --continue-on-error \
  || echo "[WARN] colcon 빌드 중 일부 에러가 발생했지만, 계속 진행합니다."
echo "[INFO] colcon 빌드 완료"

# ROS2 워크스페이스 적용
source install/setup.bash
echo "[INFO] 환경 준비 완료. launch 파일을 실행할 수 있습니다."

### main_service test
export PYTHONPATH=$PYTHONPATH:$(pwd)
python3 -m server.main_server.main_service.main              # 일반 실행 (port 9000)
# python3 -m server.main_server.main_service.main --test       # 3초 후 자동 종료
# echo "[INFO] main_service/main.py 실행 테스트 완료"

### AI 연동용 실행 (port 5001)
# python3 -m server.main_server.main_service.main --ai-test    # AI 연동용 실행 (port 5001)
# python3 -m server.ai_server.ai_modules.main
# echo "[INFO] main.py 실행 테스트 완료"

### GUI 실행
# Test용 GUI
# python3 -m viewer.mode_select 
## manager 로그인
# python3 -m viewer.login.manager_login
# GUI dashboard
# python3 -m viewer.manager.manager.dashboard_panel
## staff 로그인
# python3 -m viewer.login.staff_login
 
