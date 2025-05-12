#!/bin/bash

# 시스템 패키지 최신화
sudo apt update && sudo apt upgrade -y

# 가상환경 비활성화 (이미 활성화되어 있던 경우)
deactivate 2>/dev/null || true

# 가상환경 경로 지정
VENV_DIR=".roscars_venv"

# 가상환경 생성
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating Python virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

# 가상환경 활성화
echo "[INFO] Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Python 의존성 설치 (한 번만)
if [ ! -f "$VENV_DIR/.installed" ]; then
    echo "[INFO] Installing Python dependencies with --break-system-packages..."
    pip install --upgrade pip setuptools wheel --break-system-packages
    pip install -r requirements.txt --break-system-packages
    touch "$VENV_DIR/.installed"
    echo "[INFO] Python dependencies installed"
fi

# ROS2 환경 설정
echo "[INFO] Sourcing ROS2 setup..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

# 시스템 의존성 설치
echo "[INFO] Installing system dependencies..."
sudo apt install -y libgpiod-dev

# 이전 빌드 정리
echo "[INFO] Cleaning previous build artifacts..."
rm -rf build/ install/ log/
export PYTHONWARNINGS="ignore::UserWarning"
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

# colcon 빌드
echo "[INFO] Starting colcon build..."
colcon build --merge-install
if [ $? -ne 0 ]; then
  echo "[ERROR] Build failed"
  exit 1
fi

# ROS2 워크스페이스 적용
source install/local_setup.bash

echo "[INFO] ✅ Build complete and environment ready."
