#!/usr/bin/env python3
"""
Auto Chase Vehicle Control Script (Modularized Version)
모듈화된 자동 추격 차량 제어 스크립트
"""

import sys
import os

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from chase.control.auto_chase_controller import AutoChaseVehicleControl

def main():
    """메인 함수"""
    try:
        print("🚔 Starting Modularized Auto Chase Vehicle Control...")
        
        # 자동 추격 차량 제어 인스턴스 생성
        auto_chase = AutoChaseVehicleControl()
        
        # 실행
        auto_chase.run()
        
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error in main: {e}")

if __name__ == "__main__":
    main()

