# Unified Collision Detector

통합 충돌 감지 모듈 - 다양한 센서 데이터를 통합하여 충돌을 감지하고 추격을 시작하는 독립적인 모듈입니다.

## 🎯 주요 기능

- **다중 센서 지원**: Zenoh, CARLA 바운딩 박스, LiDAR 데이터 통합 처리
- **실시간 충돌 감지**: 차량-보행자 충돌 및 보행자 넘어짐 감지
- **콜백 기반 아키텍처**: 충돌 감지 시 콜백 함수 호출
- **설정 가능한 임계값**: 다양한 환경에 맞는 감지 파라미터 조정
- **통계 및 모니터링**: 실시간 충돌 감지 통계 제공

## 📁 파일 구조

```
main/chase/perception/
├── unified_collision_detector.py    # 메인 충돌 감지 모듈
├── test_unified_collision_detector.py  # 테스트 스크립트
└── README_unified_collision_detector.md  # 이 파일
```

## 🚀 사용법

### 1. 기본 사용법

```python
from chase.perception.unified_collision_detector import UnifiedCollisionDetector, CollisionEvent

# 충돌 감지기 초기화
detector = UnifiedCollisionDetector()

# 콜백 함수 설정
def on_collision_detected(event: CollisionEvent):
    print(f"충돌 감지: {event.description}")
    print(f"심각도: {event.severity}")

def on_chase_started(vehicle_ids):
    print(f"추격 시작: {vehicle_ids}")

detector.set_collision_callback(on_collision_detected)
detector.set_chase_callback(on_chase_started)

# 객체 데이터 처리
objects = [
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
        'world_location': [12.0, 6.0, 0.3],
        'velocity': 0.05,
        'confidence': 0.8
    }
]

# 충돌 감지 실행
collision_events = detector.process_objects(objects)

# 정리
detector.cleanup()
```

### 2. 커스텀 설정

```python
# 커스텀 설정
custom_config = {
    'collision_distance_threshold': 15.0,  # 충돌 감지 거리 (m)
    'pedestrian_fall_height_threshold': 0.3,  # 보행자 넘어짐 높이 임계값 (m)
    'bbox_overlap_tolerance': 0.05,  # 바운딩 박스 겹침 허용 오차
    'collision_cooldown': 1.0,  # 충돌 이벤트 쿨다운 (초)
    'chase_distance_threshold': 50.0,  # 추격 시작 거리 (m)
    'enable_vehicle_pedestrian_collision': True,
    'enable_pedestrian_fall_detection': True,
    'enable_bbox_overlap_detection': True
}

detector = UnifiedCollisionDetector(config=custom_config)
```

### 3. AutoChaseController와 통합

```python
# auto_chase_controller.py에서 사용
class AutoChaseVehicleControl:
    def __init__(self):
        # ... 기존 초기화 코드 ...
        self.unified_collision_detector = None
    
    def setup_modules(self):
        # 통합 충돌 감지기 초기화
        self.unified_collision_detector = UnifiedCollisionDetector()
        self.unified_collision_detector.set_collision_callback(self._on_collision_detected)
        self.unified_collision_detector.set_chase_callback(self._on_chase_started)
    
    def process_zenoh_objects(self, objects):
        if not self.unified_collision_detector:
            return
        
        # 통합 충돌 감지기로 객체 처리
        collision_events = self.unified_collision_detector.process_objects(objects)
        
        # 충돌 이벤트 처리
        if collision_events:
            print(f"🚨 {len(collision_events)} collision events detected!")
    
    def _on_collision_detected(self, collision_event: CollisionEvent):
        print(f"🚨 Collision detected: {collision_event.description}")
        # 충돌 처리 로직
    
    def _on_chase_started(self, collision_vehicles: List[str]):
        print(f"🚔 Starting chase for vehicles: {collision_vehicles}")
        # 추격 시작 로직
```

## 🔧 설정 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `collision_distance_threshold` | 10.0 | 충돌 감지 거리 (m) |
| `pedestrian_fall_height_threshold` | 0.5 | 보행자 넘어짐 높이 임계값 (m) |
| `bbox_overlap_tolerance` | 0.1 | 바운딩 박스 겹침 허용 오차 |
| `collision_cooldown` | 2.0 | 충돌 이벤트 쿨다운 (초) |
| `chase_distance_threshold` | 30.0 | 추격 시작 거리 (m) |
| `enable_vehicle_pedestrian_collision` | True | 차량-보행자 충돌 감지 활성화 |
| `enable_pedestrian_fall_detection` | True | 보행자 넘어짐 감지 활성화 |
| `enable_bbox_overlap_detection` | True | 바운딩 박스 겹침 감지 활성화 |

## 📊 충돌 이벤트 타입

### 1. Vehicle-Pedestrian Collision
- **타입**: `vehicle_pedestrian_collision`
- **조건**: 차량과 보행자가 거리 임계값 내에서 바운딩 박스가 겹침
- **심각도**: 차량 속도와 크기에 따라 결정

### 2. Pedestrian Fall
- **타입**: `pedestrian_fall`
- **조건**: 보행자의 높이가 낮음 (바닥에 누워있음)
- **심각도**: `medium`

## 📈 통계 정보

```python
stats = detector.get_statistics()
print(f"총 충돌 수: {stats['total_collisions']}")
print(f"차량-보행자 충돌: {stats['vehicle_pedestrian_collisions']}")
print(f"보행자 넘어짐: {stats['pedestrian_falls']}")
print(f"추격 이벤트: {stats['chase_events']}")
print(f"현재 추격 중: {stats['is_chasing']}")
print(f"충돌 차량: {stats['collision_vehicles']}")
```

## 🧪 테스트

```bash
# 테스트 스크립트 실행
cd /home/dongmin/SEA-ME/Eclipse/carla/Carla_ROS2_KUKSA_ZENOH_WEB/main/chase/perception
python3 test_unified_collision_detector.py
```

## 🔄 기존 코드와의 호환성

- 기존 `CollisionDetector`, `CollisionTracker` 모듈과 호환
- `auto_chase_controller.py`에서 점진적 마이그레이션 가능
- 기존 API 형식 지원 (레거시 이벤트 변환)

## 🎯 장점

1. **모듈화**: 충돌 감지 로직이 제어 코드에서 분리됨
2. **재사용성**: 다른 프로젝트에서 독립적으로 사용 가능
3. **확장성**: 새로운 충돌 감지 알고리즘 쉽게 추가 가능
4. **테스트 용이성**: 독립적인 단위 테스트 가능
5. **설정 가능성**: 다양한 환경에 맞는 파라미터 조정 가능

## 🚨 주의사항

- 충돌 감지기는 메인 스레드에서 실행되어야 함
- 콜백 함수는 빠르게 실행되어야 함 (블로킹 방지)
- 메모리 누수 방지를 위해 `cleanup()` 호출 필수
- 대량의 객체 처리 시 성능 고려 필요
