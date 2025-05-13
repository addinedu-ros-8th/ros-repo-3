#!/bin/bash

set -e

# 시스템 패키지 최신화
echo "[INFO] 시스템 패키지 최신화 중..."
sudo apt update && sudo apt upgrade -y

# 가상환경 설정
VENV_DIR=".roscars_venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] 가상환경 생성: $VENV_DIR"
    python3 -m venv "$VENV_DIR"
fi

echo "[INFO] 가상환경 활성화"
source "$VENV_DIR/bin/activate"

# Python 패키지 설치
echo "[INFO] Python 의존성 설치 중..."
pip install --upgrade pip setuptools wheel --break-system-packages
pip install -r requirements.txt --break-system-packages
pip install empy --break-system-packages

# ROS2 설정 적용
echo "[INFO] ROS2 환경 설정 적용 중..."
source /opt/ros/jazzy/setup.bash

# 시스템 패키지 설치
echo "[INFO] 시스템 의존성 설치 중..."
sudo apt install -y \
    ros-jazzy-domain-bridge \
    ros-jazzy-tf-transformations \
    libgpiod-dev

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
colcon build
echo "[INFO] 빌드 완료"

# ROS2 워크스페이스 적용
source install/setup.bash
echo "[INFO] 환경 준비 완료. launch 파일을 실행할 수 있습니다."
