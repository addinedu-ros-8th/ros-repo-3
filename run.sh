#!/bin/bash

# This script installs the required dependencies for the project.
sudo apt update && sudo apt upgrade -y

deactivate 2>/dev/null || true

# Install Python dependencies
if [ -d ".roscars_venv" ]; then
    source .roscars_venv/bin/activate
else
    python3 -m venv .roscars_venv
    source .roscars_venv/bin/activate
    pip install -r requirements.txt
fi

sudo apt install -y libgpiod-dev

rm -rf build install log

export PYTHONWARNINGS="ignore::UserWarning"


find . -type d -name "__pycache__" -exec rm -rf {} + && find . -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete

colcon build --symlink-install
