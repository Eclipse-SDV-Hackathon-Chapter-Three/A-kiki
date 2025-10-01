#!/usr/bin/env python3
"""
Test script for Unified Collision Detector
통합 충돌 감지 모듈 테스트 스크립트
"""

import sys
import os
import time
import json

# 상위 디렉토리의 모듈들을 import하기 위해 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from chase.perception.unified_collision_detector import UnifiedCollisionDetector, CollisionEvent

def test_collision_detector():
    """충돌 감지기 테스트"""
    print("🧪 Testing Unified Collision Detector...")
    
    # 충돌 감지기 초기화
    detector = UnifiedCollisionDetector()
    
    # 콜백 함수 설정
    def on_collision_detected(event: CollisionEvent):
        print(f"🚨 Collision Event: {event.event_type}")
        print(f"   Description: {event.description}")
        print(f"   Severity: {event.severity}")
        print(f"   Actor ID: {event.actor_id}")
        print(f"   Confidence: {event.confidence:.2f}")
        print()
    
    def on_chase_started(vehicle_ids):
        print(f"🚔 Chase started for vehicles: {vehicle_ids}")
        print()
    
    detector.set_collision_callback(on_collision_detected)
    detector.set_chase_callback(on_chase_started)
    
    # 테스트 데이터 생성
    test_objects = [
        {
            'actor_id': 'vehicle_001',
            'type': 'vehicle',
            'bbox_2d': {'x_min': 100, 'y_min': 100, 'x_max': 200, 'y_max': 150},
            'distance': 5.0,
            'world_location': [10.0, 5.0, 0.5],
            'velocity': 15.0,
            'confidence': 0.9
        },
        {
            'actor_id': 'pedestrian_001',
            'type': 'pedestrian',
            'bbox_2d': {'x_min': 150, 'y_min': 120, 'x_max': 180, 'y_max': 200},
            'distance': 6.0,
            'world_location': [12.0, 6.0, 0.3],  # 낮은 높이 (넘어짐)
            'velocity': 0.05,  # 매우 낮은 속도
            'confidence': 0.8
        },
        {
            'actor_id': 'vehicle_002',
            'type': 'vehicle',
            'bbox_2d': {'x_min': 300, 'y_min': 300, 'x_max': 400, 'y_max': 350},
            'distance': 20.0,
            'world_location': [30.0, 20.0, 0.5],
            'velocity': 10.0,
            'confidence': 0.85
        },
        {
            'actor_id': 'pedestrian_002',
            'type': 'pedestrian',
            'bbox_2d': {'x_min': 350, 'y_min': 320, 'x_max': 380, 'y_max': 400},
            'distance': 25.0,
            'world_location': [35.0, 25.0, 1.2],  # 정상 높이
            'velocity': 2.0,
            'confidence': 0.9
        }
    ]
    
    print("📊 Test Data:")
    for obj in test_objects:
        print(f"   {obj['type']} {obj['actor_id']}: distance={obj['distance']}m, height={obj['world_location'][2]}m")
    print()
    
    # 충돌 감지 실행
    print("🔍 Running collision detection...")
    collision_events = detector.process_objects(test_objects)
    
    # 결과 출력
    print(f"📈 Results: {len(collision_events)} collision events detected")
    print()
    
    # 통계 출력
    stats = detector.get_statistics()
    print("📊 Statistics:")
    for key, value in stats.items():
        print(f"   {key}: {value}")
    print()
    
    # 추가 테스트: 충돌 없는 상황
    print("🔍 Testing with no collision scenario...")
    safe_objects = [
        {
            'actor_id': 'vehicle_003',
            'type': 'vehicle',
            'bbox_2d': {'x_min': 500, 'y_min': 500, 'x_max': 600, 'y_max': 550},
            'distance': 50.0,
            'world_location': [50.0, 50.0, 0.5],
            'velocity': 20.0,
            'confidence': 0.9
        },
        {
            'actor_id': 'pedestrian_003',
            'type': 'pedestrian',
            'bbox_2d': {'x_min': 700, 'y_min': 700, 'x_max': 730, 'x_max': 800},
            'distance': 60.0,
            'world_location': [70.0, 70.0, 1.5],
            'velocity': 3.0,
            'confidence': 0.85
        }
    ]
    
    collision_events = detector.process_objects(safe_objects)
    print(f"📈 Safe scenario results: {len(collision_events)} collision events detected")
    print()
    
    # 정리
    detector.cleanup()
    print("✅ Test completed successfully!")

def test_configuration():
    """설정 테스트"""
    print("⚙️ Testing configuration...")
    
    # 커스텀 설정
    custom_config = {
        'collision_distance_threshold': 15.0,  # 더 넓은 감지 범위
        'pedestrian_fall_height_threshold': 0.3,  # 더 엄격한 높이 임계값
        'bbox_overlap_tolerance': 0.05,  # 더 엄격한 겹침 허용 오차
        'collision_cooldown': 1.0,  # 더 짧은 쿨다운
        'chase_distance_threshold': 50.0,  # 더 넓은 추격 범위
        'enable_vehicle_pedestrian_collision': True,
        'enable_pedestrian_fall_detection': True,
        'enable_bbox_overlap_detection': True
    }
    
    detector = UnifiedCollisionDetector(config=custom_config)
    
    print("✅ Custom configuration applied:")
    for key, value in custom_config.items():
        print(f"   {key}: {value}")
    print()
    
    detector.cleanup()

if __name__ == "__main__":
    try:
        test_collision_detector()
        print("\n" + "="*50 + "\n")
        test_configuration()
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
