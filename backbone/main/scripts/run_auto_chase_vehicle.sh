#!/bin/bash

# Auto Chase Vehicle Control Script
# 충돌 감지 시 자동으로 추격하는 차량 제어 스크립트

echo "🚔 Starting Auto Chase Vehicle Control..."

# CARLA 서버 확인
echo "🔍 Checking CARLA server..."
if ! pgrep -f "CarlaUE4" > /dev/null; then
    echo "❌ CARLA server is not running!"
    echo "   Please start CARLA server first:"
    echo "   cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15"
    echo "   ./CarlaUE4.sh"
    exit 1
fi
echo "✅ CARLA server is running"

# Zenoh 확인 (선택사항)
echo "🔍 Checking Zenoh..."
if command -v zenohd &> /dev/null; then
    if ! pgrep -f "zenohd" > /dev/null; then
        echo "🚀 Starting Zenoh daemon..."
        zenohd &
        sleep 2
    else
        echo "✅ Zenoh daemon is running"
    fi
else
    echo "⚠️  Zenoh not found - bounding box detection will work without Zenoh"
fi

# Python 의존성 확인
echo "🔍 Checking Python dependencies..."
python3 -c "import carla, pygame, cv2, numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing Python dependencies!"
    echo "   Please install required packages:"
    echo "   pip install carla pygame opencv-python numpy"
    exit 1
fi

# Zenoh Python 패키지 확인 (선택사항)
python3 -c "import zenoh" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Zenoh Python package not found - Zenoh features will be disabled"
fi

echo "✅ All dependencies checked"

# CARLA Python API 경로 설정
export PYTHONPATH="${PYTHONPATH}:/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"

# 스크립트 디렉토리로 이동
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"

echo ""
echo "🎮 CONTROLS:"
echo "  ESC - Exit"
echo "  🎯 Bounding boxes: Always ON"
echo "  🌐 Zenoh: Always ON"
echo "  🚨 Auto Chase: ON (충돌 감지 시 자동 추격)"
echo ""

# Python 스크립트 실행
python3 auto_chase_vehicle_control.py

echo "✅ Auto Chase Vehicle Control finished"
