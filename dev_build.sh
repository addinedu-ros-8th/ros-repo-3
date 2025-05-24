#!/bin/bash

set -e  # 오류 발생 시 즉시 중단

TARGET_PKG=$1

if [ -z "$TARGET_PKG" ]; then
    echo "[ERROR] 패키지명을 인자로 전달해야 합니다. 예: bash dev_build.sh manager"
    exit 1
fi

echo "[STEP 1] 가상환경 활성화 및 PYTHONPATH 등록"
source .roscars_venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(python -c "import site; print(site.getsitepackages()[0])")
export PYTHONWARNINGS="ignore::UserWarning"

echo "[STEP 2] 캐시 제거: build/$TARGET_PKG, install/$TARGET_PKG, log/$TARGET_PKG"
rm -rf build/$TARGET_PKG install/$TARGET_PKG log/$TARGET_PKG

echo "[STEP 3] __pycache__ 및 *.py[co] 제거"
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

echo "[STEP 4] AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH 정리"
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
# export PYTHONPATH=$PYTHONPATH:$(python -c "import site; print(site.getsitepackages()[0])")
export PYTHONPATH=$PYTHONPATH:$(pwd)

echo "[STEP 5] colcon 빌드 시작 (패키지: $TARGET_PKG)"
colcon build --packages-select $TARGET_PKG

echo "[STEP 6] 환경 적용 후 실행"
source install/setup.bash

echo "[STEP 7] ros2 실행 테스트"
ros2 run $TARGET_PKG main




