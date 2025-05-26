#!/bin/bash

set -e

# 0. 경로 기준 설정
ROOT_DIR=$(pwd)
SRC_DIR="$ROOT_DIR/shared_interfaces/src"
INCLUDE_DIR="$SRC_DIR/include"
PIO_DIR="$ROOT_DIR/roscars/cart_controller"   

# 1. PlatformIO 빌드
echo "[1] Building ESP32 firmware via PlatformIO..."
cd "$PIO_DIR"
pio run -e cart

# 2. ESP32 업로드
echo "[2] Uploading firmware to ESP32..."
pio run -e cart -t upload
