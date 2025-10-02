#!/bin/bash

# High-Performance CARLA Camera Processor Build Script
# Zenoh C++ + Shared Memory 기반

echo "🔨 Building High-Performance CARLA Camera Processor..."

# 디렉토리 생성
mkdir -p build
cd build

# CMake 설정
echo "📋 Configuring CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=g++ \
    -DCMAKE_CXX_STANDARD=17

if [ $? -ne 0 ]; then
    echo "❌ CMake configuration failed"
    exit 1
fi

# 빌드
echo "🔨 Building..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "❌ Build failed"
    exit 1
fi

echo "✅ Build completed successfully!"
echo "📁 Executable: build/camera_processor"
echo ""
echo "🚀 Usage:"
echo "  ./build/camera_processor"
echo ""
echo "📋 Requirements:"
echo "  - CARLA 0.9.15"
echo "  - OpenCV 4.x"
echo "  - Zenoh C++"
echo "  - Linux with shared memory support"

