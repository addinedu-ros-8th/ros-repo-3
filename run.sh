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
source /opt/ros/jazzy/setup.bash

# 시스템 의존성 설치
echo "[INFO] Installing system dependencies..."
sudo apt install -y libgpiod-dev

# 이전 빌드 정리
echo "[INFO] Cleaning previous build artifacts..."
rm -rf build install log
export PYTHONWARNINGS="ignore::UserWarning"
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete
find . -type d -name "__pycache__" -exec rm -rf {} + && find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

# Define aliases as functions for use within script
ID=214

ros_domain() {
  export ROS_DOMAIN_ID=$ID
  echo "ROS_DOMAINID is set to $ID !"
}

active_venv_jazzy() {
  source ~/venv/jazzy/bin/activate
  echo "Venv Jazzy is activated."
}

jazzy() {
  active_venv_jazzy
  source /opt/ros/jazzy/setup.bash
  ros_domain
  echo "ROS2 Jazzy is activated."
}

# Activate environment
jazzy

colcon build --symlink-install
if [ $? -ne 0 ]; then
  echo "Fail"
  exit 1
fi

source ./install/setup.bash
# source ./install/local_setup.bash

echo "[INFO] Build complete"

# ros2 run
