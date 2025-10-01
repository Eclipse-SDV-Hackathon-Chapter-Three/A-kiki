# Enhanced Police Monitoring System - Technical Documentation

## 시스템 개요 (System Overview)

이 프로젝트는 **Enhanced Police Monitoring System**으로, CARLA 시뮬레이터, ROS2, KUKSA, zenoh, 그리고 웹 기술을 통합하여 실시간 경찰차량 모니터링 시스템을 구축한 포괄적인 솔루션입니다. 시스템의 메인 파일은 `run_enhanced_police_system.py`입니다.

## 아키텍처 (Architecture)

### 실제 시스템 구성도 (Actual System Architecture)

**✅ 통합 완료**: CARLA 시뮬레이션 기반 VSS 경찰 데이터 **통합 시스템**

```
┌═══════════════════════════════════════════════════════════════════════════════════┐
│                    Enhanced Police Monitoring System                             │
│                       (run_enhanced_police_system.py)                           │
└═══════════════════════════════════════════════════════════════════════════════════┘

                                ┌─────────────────────┐
                                │   CARLA Simulator   │
                                │  (localhost:2000)   │
                                └──────────┬──────────┘
                                           │
                                           ▼
                       ┌─────────────────────────────────────────┐
                       │       carla_vehicle_spawner.py          │
                       │           (INTEGRATED)                  │
                       ├─────────────────────────────────────────┤
                       │ • Tesla Model 3 경찰차 스폰               │
                       │ • 카메라/센서 설정                        │
                       │ • CARLA 데이터 → VSS 데이터 통합 생성        │
                       └─────────────┬───────────────────────────┘
                                     │
                                     ▼
                       ┌─────────────────────────────────────────┐
                       │         ROS2 Message Layer              │
                       ├─────────────────────────────────────────┤
                       │ • /carla/hero/camera/* ←─────────────────┼──┐
                       │ • /carla/hero/vehicle_status             │  │
                       │ • police/vss_data                        │  │
                       │ • police/status                          │  │
                       └─────────┬───────────────────────┬────────┘  │
                                 │                       │           │
                                 ▼                       ▼           │
           ┌─────────────────────────────┐    ┌─────────────────────────────┐  │
           │   C++ SHM 카메라 브리지        │    │    kuksa_zenoh_bridge.py    │  │
           ├─────────────────────────────┤    ├─────────────────────────────┤  │
           │                             │    │ • VSS 스키마 검증             │  │
           │ ┌─────────────────────────┐ │    │ • 응급상황 감지              │  │
           │ │  ros2_zenoh_shm_pub     │ │    │ • zenoh Topics 발행:        │  │
           │ │  • Camera 구독          │ │    │   - police/central/vehicle/ │  │
           │ │  • SHM 전송 (67MB)     │ │    │     UNIT-001/vss           │  │
           │ │  • 실시간 스트림        │ │    │   - police/central/alerts/  │  │
           │ └─────────┬───────────────┘ │    │     emergency/UNIT-001      │  │
           │           │                 │    │   - police/central/fleet/   │  │
           │           ▼                 │    │     status                  │  │
           │ ┌─────────────────────────┐ │    └─────────────┬───────────────┘  │
           │ │  zenoh_ws_shm_sub       │ │                  │                  │
           │ │  • SHM 구독             │ │                  ▼                  │
           │ │  • WebSocket 서버       │ │                                     │
           │ │  • 포트 8080           │ │    ┌─────────────────────────────┐  │
           │ └─────────┬───────────────┘ │    │  police_websocket_bridge.py │  │
           └───────────┼─────────────────┘    ├─────────────────────────────┤  │
                       │                      │ • zenoh 구독:               │  │
                       │                      │   - police/central/*        │  │
                       │                      │ • WebSocket 서버            │  │
                       │                      │ • 포트 8081                 │  │
                       │                      └─────────────┬───────────────┘  │
                       │                                    │                  │
                       │                                    ▼                  │
                       │              ┌─────────────────────────────────────────┼──┘
                       │              │            zenoh Network                │
                       │              │          (고성능 백본)                   │
                       │              ├─────────────────────────────────────────┤
                       │              │ • SHM: 카메라 스트림 (67MB)               │
                       │              │ • Messages: VSS 경찰 데이터              │
                       │              │ • Topics:                              │
                       │              │   - police/central/vehicle/*           │
                       │              │   - police/central/alerts/*            │
                       │              │   - police/central/fleet/*             │
                       │              └─────────────────────────────────────────┘
                       │                                    │
                       └────────────────┬───────────────────┘
                                        ▼
                    ┌─────────────────────────────────────────┐
                    │          Web Interface Layer            │
                    ├─────────────────────────────────────────┤
                    │                                         │
                    │ ┌─────────────────────────────────────┐ │
                    │ │    police_dashboard_enhanced.html   │ │
                    │ ├─────────────────────────────────────┤ │
                    │ │                                     │ │
                    │ │ WebSocket 8080 ──► 실시간 카메라       │ │
                    │ │                   스트림 (CARLA)      │ │
                    │ │                         +           │ │
                    │ │ WebSocket 8081 ──► VSS 경찰 데이터    │ │
                    │ │                   (CARLA 기반)      │ │
                    │ │                         ║          │ │
                    │ │                         ▼          │ │
                    │ │ 통합 실시간 대시보드:                 │ │
                    │ │ • 차량 위치 추적                     │ │
                    │ │ • 응급상황 알림                      │ │
                    │ │ • 장비 상태 모니터링                  │ │
                    │ │ • 라이브 카메라 피드                  │ │
                    │ │ • VSS 준수 데이터 시각화              │ │
                    │ └─────────────────────────────────────┘ │
                    └─────────────────────────────────────────┘
```

## 📊 **통합된 데이터 플로우** (Integrated Data Flow)

```
CARLA Simulator
        │
        ▼
carla_vehicle_spawner.py (통합 버전)
        │
        ├─ /carla/hero/camera/* ──► ros2_zenoh_shm_pub ──► zenoh SHM ──► WebSocket(8080)
        │                                                                         │
        ├─ police/vss_data ────────► kuksa_zenoh_bridge.py ──► zenoh Topics ──► WebSocket(8081)
        │                                    │                        │              │
        └─ police/status ──────────────────► │                        ▼              │
                                           │              police/central/vehicle/*    │
                                           │              police/central/alerts/*     │
                                           ▼              police/central/fleet/*      │
                                   police_websocket_bridge.py                      │
                                           │                                         │
                                           └─────────────────────────────────────────┘
                                                                  │
                                                                  ▼
                                                 police_dashboard_enhanced.html
                                                 (통합 실시간 경찰 모니터링)
```

## 핵심 개선사항 (Key Improvements)

### 1. **✅ 통합된 데이터 플로우**
- **통합 CARLA-VSS**: 실제 차량 시뮬레이션 데이터를 기반으로 VSS 데이터 생성
- **실시간 연동**: CARLA 차량 상태가 VSS 경찰 데이터에 직접 반영
- **일관성 보장**: 위치, 속도, 상태가 모든 데이터 스트림에서 동일

### 2. **✅ CARLA 기반 VSS 통합**
- `carla_vehicle_spawner.py`: 통합된 차량 스폰 + VSS 데이터 생성
- `police_vss_data_publisher.py`: **더 이상 사용하지 않음 (DEPRECATED)**
- 실제 CARLA 차량 데이터 → VSS 포맷 실시간 변환
- 경찰 시나리오가 실제 차량 행동과 연동

### 3. **zenoh의 이중 역할**
- **일반 메시지**: VSS 경찰 데이터 라우팅 (`police/*` 토픽)
- **SHM (Shared Memory)**: 고성능 카메라 스트림 (67MB 용량)
- 하나의 zenoh 라우터가 두 가지 통신 방식을 동시 지원

### 4. **웹 인터페이스 통합점**
- 포트 8080: 실시간 카메라 스트림 (CARLA 출처)
- 포트 8081: VSS 경찰 데이터 (시뮬레이션 출처)
- `police_dashboard_enhanced.html`에서 두 스트림 통합 표시

### 핵심 컴포넌트 (Core Components)

#### 1. 메인 런처 (Main Launcher)
**파일**: `run_enhanced_police_system.py`
- **클래스**: `EnhancedPoliceSystemLauncher`
- **역할**: 전체 시스템 오케스트레이션 및 프로세스 관리
- **주요 기능**:
  - 의존성 검사 (CARLA, ROS2, zenoh, Python 패키지, C++ 바이너리)
  - 순차적 컴포넌트 시작 (zenoh 라우터 → CARLA 스폰너 → VSS 퍼블리셔 → 브리지들)
  - 프로세스 상태 모니터링 및 오류 처리
  - graceful shutdown 처리

#### 2. VSS 데이터 퍼블리셔 (VSS Data Publisher)
**파일**: `police_vss_data_publisher.py`
- **클래스**: `PoliceVSSDataPublisher`
- **표준**: VSS (Vehicle Signal Specification) 준수
- **데이터 구조**:
  ```python
  {
    "Vehicle": {
      "VehicleIdentification": {...},
      "CurrentLocation": {...},
      "Police": {
        "Status": {...},
        "Emergency": {...},
        "Equipment": {...},
        "Officer": {...}
      },
      "Body": {...},
      "Powertrain": {...},
      "Service": {...},
      "OBD": {...}
    }
  }
  ```

#### 3. KUKSA-zenoh 브리지 (KUKSA-zenoh Bridge)
**파일**: `kuksa_zenoh_bridge.py`
- **클래스**: `KuksaZenohBridge`
- **기능**:
  - VSS 데이터 유효성 검증 (JSON Schema 기반)
  - 응급 상황 감지 및 알림 시스템
  - 실시간 데이터 변환 및 라우팅
  - 성능 모니터링 및 통계

#### 4. 웹소켓 브리지 (WebSocket Bridge)
**파일**: `police_websocket_bridge.py`
- **클래스**: `PoliceWebSocketBridge`
- **포트**: 8081 (경찰 VSS 데이터)
- **기능**:
  - zenoh 구독 및 WebSocket 브로드캐스트
  - 클라이언트 연결 관리
  - 메시지 큐 기반 효율적 브로드캐스팅
  - 응급 알림 우선 처리

#### 5. CARLA 차량 스폰너 (CARLA Vehicle Spawner)
**파일**: `carla_vehicle_spawner.py`
- **클래스**: `CarlaVehicleSpawner`
- **기능**:
  - Tesla Model 3 차량 스폰 (경찰 역할)
  - 카메라 및 LiDAR 센서 설정
  - 자율주행 활성화
  - ROS2 토픽 퍼블리싱

## 기술적 상세 (Technical Details)

### 1. 통합된 데이터 플로우 (Integrated Data Flow)

**✅ 통합된 CARLA+VSS 시스템**:

#### 주요 데이터 경로 (Main Data Paths)
```
1. 카메라 스트림 경로:
   CARLA → carla_vehicle_spawner.py → /carla/hero/camera/* →
   ros2_zenoh_shm_pub → zenoh SHM → zenoh_ws_shm_sub → WebSocket(8080)

2. VSS 데이터 경로:
   CARLA → carla_vehicle_spawner.py → police/vss_data →
   kuksa_zenoh_bridge.py → police/central/* → police_websocket_bridge.py → WebSocket(8081)

3. 웹 대시보드 통합:
   WebSocket(8080) + WebSocket(8081) → police_dashboard_enhanced.html
```

**✅ 시스템 통합의 핵심 특징**:
- **단일 소스 진실성**: `carla_vehicle_spawner.py`가 CARLA와 VSS 데이터를 모두 생성
- **완전한 동기화**: 차량 위치, 속도, 상태가 모든 데이터 스트림에서 일치
- **실시간 연동**: CARLA 차량 행동이 VSS 경찰 시나리오에 즉시 반영
- **이중 zenoh 활용**: SHM(카메라)과 Messages(VSS) 동시 지원
- **웹 통합**: 두 독립적 데이터 스트림이 하나의 대시보드에서 통합 표시

### 2. 네트워크 프로토콜 (Network Protocols)

#### zenoh 토픽 구조 (zenoh Topic Structure)
```
police/{district}/vehicle/{unit_id}/vss        # VSS 데이터
police/{district}/vehicle/{unit_id}/status     # 상태 데이터
police/{district}/alerts/emergency/{unit_id}   # 응급 알림
police/{district}/fleet/status                 # 함대 상태
police/{district}/bridge/heartbeat             # 브리지 상태
```

#### WebSocket 메시지 타입 (WebSocket Message Types)
```javascript
{
  type: "vss_data" | "status" | "emergency_alert" | "fleet_status" | "heartbeat",
  unit_id: string,
  data: object,
  timestamp: number
}
```

### 3. VSS 스키마 검증 (VSS Schema Validation)

**파일**: `vss_police_schema.json`
- JSON Schema Draft-07 표준
- 경찰차량 특화 데이터 구조 정의
- 실시간 데이터 유효성 검증

### 4. C++ 고성능 컴포넌트 (High-Performance C++ Components)

#### 빌드 시스템 (Build System)
- **CMake**: `CMakeLists.txt`
- **Makefile**: 대안 빌드 시스템
- **의존성**: ROS2 Humble, zenoh-c, Boost.Beast, nlohmann/json

#### 컴파일된 바이너리 (Compiled Binaries)
```
build/ros2_zenoh_shm_pub     # ROS2 → zenoh SHM 퍼블리셔
build/zenoh_ws_shm_sub       # zenoh SHM → WebSocket 구독자
```

### 5. 응급 상황 처리 (Emergency Handling)

#### 응급 시나리오 (Emergency Scenarios)
- **교통 단속** (traffic_stop): CODE1, 사이렌 OFF
- **추격전** (pursuit): CODE3, 사이렌 ON, 고속
- **가정폭력 신고** (domestic_call): CODE2, 사이렌 OFF
- **일반 순찰** (routine_patrol): 모든 경고등 OFF

#### 패닉 버튼 시스템 (Panic Button System)
- 0.001% 확률로 임의 발생 (시뮬레이션)
- 즉시 CRITICAL 우선순위 알림 발송
- 5초 후 자동 리셋

## 설치 및 설정 (Installation & Setup)

### 시스템 요구사항 (System Requirements)

#### 소프트웨어 의존성 (Software Dependencies)
```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Python 패키지
pip install rclpy zenoh jsonschema websockets opencv-python

# 시스템 패키지
sudo apt install nlohmann-json3-dev libboost-dev zenoh-c-dev

# CARLA 시뮬레이터
# /opt/carla-simulator/ 또는 /home/seame/PythonAPI/
```

#### 하드웨어 요구사항 (Hardware Requirements)
- **CPU**: 최소 4코어 (권장 8코어)
- **RAM**: 최소 8GB (권장 16GB)
- **GPU**: NVIDIA GTX 1060 이상 (CARLA용)
- **네트워크**: 기가비트 이더넷

### 빌드 과정 (Build Process)

#### C++ 컴포넌트 빌드
```bash
# CMAKE 방식
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Makefile 방식
make all
```

#### 설정 파일 (Configuration Files)
```json
// config/police_system_config.json (선택적)
{
  "camera_shm_capacity": 67108864,
  "police_district": "central",
  "enable_vss_validation": true,
  "enable_emergency_alerts": true,
  "carla_host": "localhost",
  "carla_port": 2000,
  "police_units": [...],
  "dashboard_auto_open": true,
  "log_level": "INFO"
}
```

## 실행 방법 (Execution)

### 전체 시스템 시작 (Full System Startup)
```bash
python3 run_enhanced_police_system.py
```

### 개별 컴포넌트 실행 (Individual Component Execution)
```bash
# VSS 데이터 퍼블리셔
python3 police_vss_data_publisher.py

# KUKSA-zenoh 브리지
python3 kuksa_zenoh_bridge.py

# 웹소켓 브리지
python3 police_websocket_bridge.py --port 8081 --district central

# CARLA 차량 스폰너
python3 carla_vehicle_spawner.py

# C++ SHM 퍼블리셔
./build/ros2_zenoh_shm_pub --capacity 67108864

# C++ 웹소켓 구독자
./build/zenoh_ws_shm_sub --port 8080
```

### 시스템 종료 (System Shutdown)
- `Ctrl+C`로 graceful shutdown
- 모든 프로세스 자동 정리
- CARLA 액터 삭제

## 모니터링 및 디버깅 (Monitoring & Debugging)

### 로그 시스템 (Logging System)
```python
# 로그 레벨: DEBUG, INFO, WARNING, ERROR
# 파일 출력: enhanced_police_system_{timestamp}.log
# 콘솔 출력: 실시간 상태 표시
```

### 성능 지표 (Performance Metrics)
```python
{
  "messages_received": int,
  "messages_published": int,
  "validation_errors": int,
  "emergency_alerts": int,
  "uptime_seconds": float,
  "active_units_count": int
}
```

### API 엔드포인트 (API Endpoints)
```
카메라 스트림:    ws://localhost:8080
경찰 VSS 데이터:  ws://localhost:8081
대시보드:        file://police_dashboard_enhanced.html
```

## 확장성 (Extensibility)

### 새로운 센서 추가 (Adding New Sensors)
1. CARLA 센서 정의 (`carla_vehicle_spawner.py`)
2. VSS 스키마 확장 (`vss_police_schema.json`)
3. 데이터 퍼블리셔 업데이트 (`police_vss_data_publisher.py`)

### 다중 지역 지원 (Multi-District Support)
```python
# 지역별 zenoh 토픽 분리
f"police/{district}/vehicle/{unit_id}/..."
```

### 사용자 정의 시나리오 (Custom Scenarios)
```python
# emergency_scenarios 배열에 새 시나리오 추가
{
  "name": "custom_scenario",
  "duration": 300,
  "lightbar": "CUSTOM",
  "siren": True,
  "availability": "BUSY",
  "call_type": "CUSTOM CALL",
  "priority": "MEDIUM"
}
```

## 보안 고려사항 (Security Considerations)

### 네트워크 보안 (Network Security)
- zenoh 라우터 인증 설정 권장
- WebSocket 연결 HTTPS/WSS 업그레이드
- 방화벽 포트 제한 (8080, 8081)

### 데이터 검증 (Data Validation)
- JSON Schema 기반 VSS 검증
- 입력 데이터 살균 (sanitization)
- 오류 처리 및 로깅

## 문제 해결 (Troubleshooting)

### 일반적인 문제 (Common Issues)

#### CARLA 연결 실패
```bash
# CARLA 서버 상태 확인
netstat -tlnp | grep :2000

# CARLA 재시작
cd /opt/carla-simulator && ./CarlaUE4.sh
```

#### zenoh 라우터 문제
```bash
# zenoh 프로세스 확인
pgrep -f zenohd

# 수동 라우터 시작
zenohd --cfg 'transport: {shared_memory: {enabled: true}}'
```

#### 빌드 오류
```bash
# 의존성 재설치
sudo apt update && sudo apt install -y nlohmann-json3-dev libboost-dev

# 빌드 디렉토리 클린
rm -rf build && mkdir build
```

### 성능 최적화 (Performance Optimization)

#### SHM 용량 조정 (SHM Capacity Tuning)
```bash
# 기본값: 64MB
./ros2_zenoh_shm_pub --capacity 134217728  # 128MB
```

#### CPU 친화성 설정 (CPU Affinity)
```bash
# 특정 CPU 코어에 바인딩
taskset -c 0-3 python3 run_enhanced_police_system.py
```

## 라이선스 및 기여 (License & Contributing)

### 라이선스 (License)
- **프로젝트**: MIT License
- **zenoh**: Apache 2.0 / Eclipse Public License 2.0
- **ROS2**: Apache 2.0
- **CARLA**: MIT License

### 기여 방법 (Contributing)
1. 이슈 생성 및 논의
2. 포크 및 브랜치 생성
3. 테스트 코드 작성
4. 풀 리퀘스트 제출

---

## 결론 (Conclusion)

이 **Enhanced Police Monitoring System**은 최신 자율주행 시뮬레이션, 로봇공학, 통신 기술을 통합하여 실시간 경찰차량 모니터링을 위한 완전한 솔루션을 제공합니다.

### 핵심 아키텍처 혁신점

1. **분리된 데이터 플로우 설계**: CARLA 물리 시뮬레이션과 VSS 경찰 업무 데이터를 완전히 분리하여 각각 최적화
2. **이중 zenoh 활용**: SHM(고성능 카메라)와 일반 메시지(VSS 데이터)를 동시 지원
3. **웹 통합 아키텍처**: 독립적인 데이터 스트림을 웹 대시보드에서 실시간 통합

### 기술적 완성도

- **VSS 표준 준수**: 표준화된 경찰차량 데이터 구조
- **고성능 C++ 컴포넌트**: 67MB SHM 용량의 실시간 카메라 스트리밍
- **확장 가능한 아키텍처**: 멀티 지역, 다중 센서 지원 가능
- **견고한 프로세스 관리**: graceful shutdown, 오류 복구, 성능 모니터링

시스템의 핵심인 `run_enhanced_police_system.py`는 **3개의 독립적인 데이터 플로우**를 안정적으로 오케스트레이션하며, 각 컴포넌트의 고유한 역할을 보장합니다. 이러한 분리된 설계는 연구, 교육, 그리고 상용 애플리케이션 모든 영역에서 유연하게 활용 가능한 견고한 플랫폼을 제공합니다.