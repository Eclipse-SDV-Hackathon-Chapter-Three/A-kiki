# Zenoh 기반 추격 시스템

## 🎯 개요

Zenoh를 통한 모듈화된 추격 시스템으로, 실시간 통신과 분산 처리를 지원합니다.

## 🏗️ 시스템 아키텍처

### 모듈 구조
```
chase/
├── communication/
│   ├── __init__.py
│   └── zenoh_manager.py          # Zenoh 통신 관리
├── perception/
│   ├── bounding_box_detector.py  # 바운딩 박스 감지
│   ├── collision_detector.py     # 충돌 감지
│   └── zenoh_detection_publisher.py  # Zenoh 감지 발행
├── planning/
│   ├── chase_planner.py          # 추격 계획
│   └── zenoh_chase_controller.py # Zenoh 추격 제어
└── README_ZENOH_SYSTEM.md
```

### Zenoh 토픽 구조
```
carla/police/
├── detection          # 감지 데이터
├── collision          # 충돌 이벤트
├── chase_command      # 추격 명령
├── vehicle_status     # 차량 상태
├── pedestrian_status  # 보행자 상태
└── emergency          # 비상 상황
```

## 🚀 실행 방법

### 1. 사전 요구사항
```bash
# CARLA 서버 실행
cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15
./CarlaUE4.sh

# Zenoh 데몬 실행
zenohd

# Python 의존성 설치
pip install carla pygame zenoh-python
```

### 2. 보행자 시나리오 실행
```bash
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"
./run_zenoh_pedestrian_scenario.sh
```

### 3. 추격 시스템 실행
```bash
cd "/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/new_examples /police/main/scripts"
./run_zenoh_chase_system.sh
```

## 🎮 제어 키

### 추격 시스템
- **ESC**: 종료
- **C**: 수동 충돌 트리거
- **S**: 추격 시작
- **T**: 추격 중지

### 보행자 시나리오
- **Ctrl+C**: 종료

## 📡 Zenoh 통신

### 발행 토픽
- `carla/police/detection`: 감지된 객체 정보
- `carla/police/collision`: 충돌 이벤트
- `carla/police/vehicle_status`: 차량 상태
- `carla/police/pedestrian_status`: 보행자 상태

### 구독 토픽
- `carla/police/chase_command`: 추격 명령
- `carla/police/emergency`: 비상 상황

## 🔧 주요 기능

### 1. 실시간 감지
- 바운딩 박스 감지
- 충돌 감지 (넘어진 보행자)
- 객체 추적

### 2. Zenoh 통신
- 토픽 기반 메시지 전달
- 실시간 데이터 스트리밍
- 분산 시스템 지원

### 3. 모듈화 설계
- 독립적인 모듈들
- 쉬운 확장성
- 재사용 가능한 컴포넌트

### 4. 고급 추격
- CARLA agents 통합
- 차선 유지
- 회피 기능
- 예측 추격

## 🛠️ 개발자 가이드

### 새로운 모듈 추가
1. 적절한 디렉토리에 모듈 생성
2. `__init__.py` 파일 추가
3. Zenoh 토픽 정의
4. 발행/구독 로직 구현

### 토픽 추가
```python
# zenoh_manager.py에 토픽 추가
self.topics = {
    'new_topic': 'carla/police/new_topic',
    # ... 기존 토픽들
}
```

### 메시지 형식
```python
# 표준 메시지 형식
message = {
    'timestamp': time.time(),
    'data': {...},
    'metadata': {...}
}
```

## 🐛 문제 해결

### Zenoh 연결 실패
```bash
# Zenoh 데몬 상태 확인
ps aux | grep zenoh

# Zenoh 데몬 재시작
pkill zenohd
zenohd
```

### CARLA 연결 실패
```bash
# CARLA 서버 상태 확인
ps aux | grep CarlaUE4

# CARLA 서버 재시작
cd /home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15
./CarlaUE4.sh
```

### 모듈 import 오류
```bash
# Python 경로 확인
export PYTHONPATH="${PYTHONPATH}:/home/dongmin/SEA-ME/Eclipse/carla/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"
```

## 📊 성능 최적화

### 발행 간격 조정
```python
# zenoh_detection_publisher.py
self.publish_interval = 0.1  # 100ms
```

### 업데이트 간격 조정
```python
# zenoh_chase_controller.py
if current_time - self.last_update_time < 0.05:  # 50ms
    return
```

## 🔮 향후 계획

1. **다중 차량 지원**: 여러 추격 차량 동시 제어
2. **AI 기반 의사결정**: 머신러닝 모델 통합
3. **웹 대시보드**: 실시간 모니터링 인터페이스
4. **로깅 시스템**: 상세한 로그 및 분석
5. **테스트 프레임워크**: 자동화된 테스트 시스템

## 📝 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.
