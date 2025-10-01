# CARLA Bounding Box Detection with Zenoh

CARLA에서 바운딩 박스 감지 기능과 Zenoh를 통한 데이터 전송을 구현한 시스템입니다.

## 기능

- **CARLA 내장 바운딩 박스 감지**: CARLA의 내장 기능을 사용하여 차량과 보행자의 3D 바운딩 박스를 감지
- **2D 투영**: 3D 바운딩 박스를 카메라 뷰의 2D 좌표로 투영
- **실시간 표시**: 카메라 뷰에 바운딩 박스를 실시간으로 표시
- **Zenoh 통신**: Zenoh를 통해 바운딩 박스 데이터를 실시간으로 전송/수신
- **다중 센서 지원**: 카메라, LiDAR, 레이더 데이터 융합

## 파일 구조

```
chase/perception/
├── bounding_box_detector.py          # CARLA 바운딩 박스 감지기
├── zenoh_bounding_box_publisher.py   # Zenoh 퍼블리셔/구독자
├── object_detector.py                # 기존 객체 감지기
└── sensor_manager.py                 # 기존 센서 관리자
```

## 의존성

### Python 패키지
```bash
pip install carla pygame opencv-python numpy zenoh
```

### CARLA
- CARLA 0.9.15
- CARLA 서버가 실행 중이어야 함

### Zenoh
- Zenoh daemon이 실행 중이어야 함
- 기본 설정: `tcp/127.0.0.1:7447`

## 사용법

### 1. CARLA 서버 시작
```bash
cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15
./CarlaUE4.sh
```

### 2. Zenoh daemon 시작 (선택사항)
```bash
zenohd
```

### 3. 바운딩 박스 카메라 데모 실행
```bash
cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples\ /police/main/scripts
./run_bounding_box_demo.sh
```

### 4. Zenoh 구독자 실행 (별도 터미널)
```bash
cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples\ /police/main/scripts
./run_zenoh_subscriber.sh
```

## 컨트롤

### 카메라 데모 컨트롤
- `ESC`: 종료
- `B`: 바운딩 박스 표시 토글
- `Z`: Zenoh 전송 토글
- `H`: HUD 표시 토글

## API 사용법

### BoundingBoxDetector 사용
```python
from perception.bounding_box_detector import BoundingBoxDetector

# 감지기 생성
detector = BoundingBoxDetector(world, camera)

# 바운딩 박스 감지
detections = detector.detect_pedestrians_and_vehicles(max_distance=100.0)

# 이미지에 바운딩 박스 그리기
image_with_boxes = detector.draw_bounding_boxes_on_image(image)

# Zenoh 전송용 데이터 포맷
zenoh_data = detector.get_detection_data_for_zenoh()
```

### ZenohBoundingBoxPublisher 사용
```python
from perception.zenoh_bounding_box_publisher import ZenohBoundingBoxPublisher, create_zenoh_config

# 설정 생성
config = create_zenoh_config(topic='carla/bounding_boxes', publish_rate=10.0)

# 퍼블리셔 생성
publisher = ZenohBoundingBoxPublisher(config)

# 연결 및 시작
if publisher.connect():
    publisher.start_publishing()
    
    # 데이터 전송
    publisher.publish_bounding_boxes(detection_data)
```

### ZenohBoundingBoxSubscriber 사용
```python
from perception.zenoh_bounding_box_publisher import ZenohBoundingBoxSubscriber

# 구독자 생성
subscriber = ZenohBoundingBoxSubscriber(config)

# 콜백 설정
def on_data(data):
    print(f"Received {len(data['detections'])} detections")

subscriber.set_callback(on_data)

# 연결 및 시작
if subscriber.connect():
    subscriber.start_subscribing()
```

## 데이터 포맷

### 바운딩 박스 탐지 데이터
```json
{
  "timestamp": 1234567890.123,
  "camera_id": "camera_001",
  "detections": [
    {
      "type": "vehicle",
      "class_name": "car",
      "bbox_2d": {
        "x_min": 100,
        "y_min": 100,
        "x_max": 200,
        "y_max": 150,
        "width": 100,
        "height": 50,
        "center_x": 150,
        "center_y": 125
      },
      "bbox_3d": [...],
      "world_location": [10.5, 20.3, 1.2],
      "distance": 25.7,
      "confidence": 0.95,
      "actor_id": 12345,
      "velocity": {
        "x": 5.2,
        "y": 0.1,
        "z": 0.0,
        "magnitude": 5.2
      }
    }
  ]
}
```

## 설정 옵션

### Zenoh 설정
```python
config = {
    'mode': 'peer',  # 또는 'client'
    'locators': ['tcp/127.0.0.1:7447'],
    'topic': 'carla/bounding_boxes',
    'publish_rate': 10.0,  # Hz
    'timeout': 5.0
}
```

### 바운딩 박스 감지 설정
```python
# 최대 감지 거리 (미터)
max_distance = 100.0

# 카메라 설정
camera_location = carla.Location(x=1.5, z=1.4)
camera_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)
image_size = (1280, 720)
fov = 90
```

## 문제 해결

### CARLA 연결 실패
- CARLA 서버가 실행 중인지 확인
- 포트 2000이 사용 가능한지 확인
- 방화벽 설정 확인

### Zenoh 연결 실패
- Zenoh daemon이 실행 중인지 확인
- 네트워크 설정 확인
- 로케이터 주소 확인

### 바운딩 박스가 표시되지 않음
- 카메라가 올바르게 설정되었는지 확인
- 차량이나 보행자가 카메라 시야 내에 있는지 확인
- 최대 감지 거리 설정 확인

### 성능 문제
- 퍼블리시 주기 조정
- 최대 감지 거리 제한
- 불필요한 센서 비활성화

## 확장 가능성

1. **다중 카메라 지원**: 여러 카메라에서 동시에 바운딩 박스 감지
2. **딥러닝 모델 통합**: YOLO, R-CNN 등과 결합
3. **추적 기능**: 객체 추적 및 궤적 예측
4. **데이터베이스 저장**: 탐지 결과를 데이터베이스에 저장
5. **웹 대시보드**: 실시간 웹 인터페이스 제공

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

