#!/bin/bash

# Simple Chase System Runner
# 단순 추격 시스템 실행 스크립트

echo "🚗 Starting Simple Chase System..."

# CARLA 서버가 실행 중인지 확인
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "❌ CARLA server is not running!"
    echo "Please start CARLA server first:"
    echo "  cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15"
    echo "  ./CarlaUE4.sh"
    exit 1
fi

# Python 의존성 확인
echo "🔍 Checking Python dependencies..."
python3 -c "import carla, pygame, numpy, cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing Python dependencies!"
    echo "Please install required packages:"
    echo "  pip install carla pygame numpy opencv-python"
    exit 1
fi

# 스크립트 디렉토리로 이동
cd "$(dirname "$0")"

# 단순 추격 시스템 실행
echo "🚀 Running Simple Chase System..."
python3 simple_chase_system.py

echo "✅ Simple Chase System finished"
