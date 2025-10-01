#!/bin/bash

# Manual Vehicle Control Script
# 수동 차량 제어를 위한 실행 스크립트

echo "🚗 Starting Manual Vehicle Control..."

# CARLA Python API 경로 설정
export PYTHONPATH="${PYTHONPATH}:/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"

# 스크립트 디렉토리로 이동
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"

# Python 스크립트 실행
python3 manual_vehicle_control.py

echo "✅ Manual Vehicle Control finished"
