#!/bin/bash

set -e

# 시스템 패키지 최신화
sudo apt update && sudo apt upgrade -y

# 가상환경 경로 지정
VENV_DIR=".roscars_venv"

# 가상환경 생성 및 의존성 설치
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating Python virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

echo "[INFO] Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Python 의존성 설치
echo "[INFO] Installing Python dependencies..."
pip install --upgrade pip setuptools wheel --break-system-packages
pip install -r requirements.txt --break-system-packages
pip install empy --break-system-packages  # ROS2 IDL 빌드용

# ROS2 환경 설정
echo "[INFO] Sourcing ROS2 setup..."
source /opt/ros/jazzy/setup.bash

# 시스템 의존성 설치
echo "[INFO] Installing system dependencies..."
sudo apt install -y \
    ros-jazzy-domain-bridge \
    ros-jazzy-tf-transformations \  # 추가됨
    libgpiod-dev

# 빌드 정리
echo "[INFO] Cleaning previous build artifacts..."
rm -rf build/ install/ log/
export PYTHONWARNINGS="ignore::UserWarning"
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

# 빌드 환경 재적용
source "$VENV_DIR/bin/activate"
source /opt/ros/jazzy/setup.bash

# AMENT/CMAKE 환경 변수 경고 제거용 경로 필터링 함수
filter_existing_paths() {
    local input="$1"
    local result=""
    IFS=':' read -ra paths <<< "$input"
    for path in "${paths[@]}"; do
        if [ -d "$path" ]; then
            result+="${path}:"
        fi
    done
    echo "${result%:}"  # 마지막 콜론 제거
}

echo "[INFO] Cleaning invalid AMENT_PREFIX_PATH and CMAKE_PREFIX_PATH..."
export AMENT_PREFIX_PATH=$(filter_existing_paths "$AMENT_PREFIX_PATH")
export CMAKE_PREFIX_PATH=$(filter_existing_paths "$CMAKE_PREFIX_PATH")
echo "[INFO] AMENT_PREFIX_PATH => $AMENT_PREFIX_PATH"
echo "[INFO] CMAKE_PREFIX_PATH => $CMAKE_PREFIX_PATH"

# 빌드 시작
echo "[INFO] Starting colcon build..."
colcon build
echo "[INFO] Build complete"

# ROS2 워크스페이스 적용
source install/setup.bash
echo "[INFO] Environment ready. You can now run your launch files."
