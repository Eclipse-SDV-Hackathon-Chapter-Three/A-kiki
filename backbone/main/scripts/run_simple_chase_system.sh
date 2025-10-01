#!/bin/bash

# Simple Chase System Script
# 간단하고 효과적인 추격 시스템 실행

echo "🚔 Starting Simple Chase System..."

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

# Python 의존성 확인
echo "🔍 Checking Python dependencies..."
python3 -c "import carla, pygame" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing Python dependencies!"
    echo "   Please install required packages:"
    echo "   pip install carla pygame"
    exit 1
fi
echo "✅ All dependencies checked"

# CARLA Python API 경로 설정
export PYTHONPATH="${PYTHONPATH}:/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"

# 스크립트 디렉토리로 이동
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"

echo ""
echo "🎮 CONTROLS:"
echo "  ESC - Exit"
echo "  C - Manual collision trigger"
echo "  S - Start chase"
echo "  T - Stop chase"
echo "  🎯 Bounding boxes: Always ON"
echo ""

# Python 스크립트 실행
python3 simple_chase_system.py

echo "✅ Simple Chase System finished"
