#!/bin/bash

# SHM-based High-Performance Camera System
# C++ 프로세서 + Python 뷰어

echo "🚀 Starting SHM-based High-Performance Camera System"
echo "=================================================="

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

# Zenoh 확인
echo "🔍 Checking Zenoh..."
if ! command -v zenohd &> /dev/null; then
    echo "❌ Zenoh daemon not found!"
    echo "   Please install Zenoh first:"
    echo "   https://zenoh.io/docs/getting-started/quick-start/"
    exit 1
fi

if ! pgrep -f "zenohd" > /dev/null; then
    echo "🚀 Starting Zenoh daemon..."
    zenohd &
    sleep 2
else
    echo "✅ Zenoh daemon is running"
fi

# Python 기반 고성능 시스템 사용
echo "🔍 Using Python-based high-performance system..."
PYTHON_SYSTEM_PATH="/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts/high_performance_camera_system.py"

if [ ! -f "$PYTHON_SYSTEM_PATH" ]; then
    echo "❌ Python high-performance system not found"
    exit 1
fi

# Python 의존성 확인
echo "🔍 Checking Python dependencies..."
python3 -c "import zenoh, cv2, numpy, pygame" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing Python dependencies!"
    echo "   Please install required packages:"
    echo "   pip install zenoh opencv-python numpy pygame"
    exit 1
fi

echo "✅ All dependencies checked"

# 환경 변수 설정
export PYTHONPATH="${PYTHONPATH}:/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"

# 스크립트 디렉토리로 이동
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"

echo ""
echo "🎬 Starting High-Performance Camera System..."
echo ""
echo "Components:"
echo "  🐍 Python High-Performance Processor"
echo "  🖥️  Python SHM Viewer (Display)"
echo "  🌐 Zenoh Communication"
echo "  💾 Shared Memory (Zero-copy)"
echo ""
echo "🎮 Controls:"
echo "  ESC - Exit viewer"
echo ""

# Python 고성능 시스템 시작
echo "🚀 Starting Python high-performance camera system..."
python3 high_performance_camera_system.py &
SYSTEM_PID=$!

# 신호 처리
cleanup() {
    echo ""
    echo "🛑 Shutting down High-Performance Camera System..."
    
    # 프로세스 종료
    kill $SYSTEM_PID 2>/dev/null
    
    # 공유 메모리 정리
    rm -f /dev/shm/carla_camera_shm
    
    echo "✅ Cleanup completed"
    exit 0
}

# 시그널 핸들러 설정
trap cleanup SIGINT SIGTERM

# 프로세스 모니터링
echo "📊 Monitoring system..."
while true; do
    if ! kill -0 $SYSTEM_PID 2>/dev/null; then
        echo "❌ High-performance system stopped unexpectedly"
        cleanup
    fi
    
    sleep 1
done
