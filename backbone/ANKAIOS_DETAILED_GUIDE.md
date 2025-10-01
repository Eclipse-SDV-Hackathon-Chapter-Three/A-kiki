# Ankaios 상세 가이드 (Detailed Guide)

Eclipse Ankaios를 사용한 Enhanced Police Monitoring System의 완벽한 이해를 위한 상세 가이드입니다.

---

## 📚 목차

1. [Ankaios란 무엇인가?](#1-ankaios란-무엇인가)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [작동 원리](#3-작동-원리)
4. [워크로드 생명주기](#4-워크로드-생명주기)
5. [의존성 관리](#5-의존성-관리)
6. [재시작 정책](#6-재시작-정책)
7. [통신 프로토콜](#7-통신-프로토콜)
8. [실전 시나리오](#8-실전-시나리오)
9. [문제 해결](#9-문제-해결)
10. [고급 활용](#10-고급-활용)

---

## 1. Ankaios란 무엇인가?

### 1.1 개요

**Eclipse Ankaios**는 자동차 산업을 위해 특별히 설계된 **워크로드 오케스트레이션 플랫폼**입니다.

```
┌────────────────────────────────────────────┐
│  Kubernetes (쿠버네티스)                    │
│  - 클라우드/데이터센터용                     │
│  - 무겁고 복잡함                            │
│  - 자동차에는 과도한 기능                    │
└────────────────────────────────────────────┘
                  vs
┌────────────────────────────────────────────┐
│  Eclipse Ankaios (안카이오스)               │
│  - 자동차 HPC 플랫폼용                      │
│  - 가볍고 효율적                            │
│  - 자동차에 최적화                          │
└────────────────────────────────────────────┘
```

### 1.2 왜 Ankaios를 사용하는가?

#### 전통적인 방식 (`run_enhanced_police_system.py`)

```python
# Python 스크립트로 직접 프로세스 관리
import subprocess

# 각 프로세스를 subprocess로 시작
zenoh_process = subprocess.Popen(['zenohd'])
carla_process = subprocess.Popen(['python3', 'carla_spawner.py'])
# ... 더 많은 프로세스들

# 문제점:
# 1. 프로세스가 죽으면? → 수동으로 재시작
# 2. 의존성 관리? → sleep()으로 시간 대기
# 3. 여러 컴퓨터에서 실행? → 불가능
# 4. 프로세스 격리? → 없음 (같은 시스템 공유)
```

#### Ankaios 방식

```yaml
# YAML 파일로 선언적 관리
workloads:
  zenoh-router:
    restartPolicy: ALWAYS  # 자동 재시작
    dependencies: {}       # 의존성 명시

  carla-spawner:
    restartPolicy: ON_FAILURE
    dependencies:
      zenoh-router: ADD_COND_RUNNING  # 자동 순서 관리

# 장점:
# 1. 자동 재시작
# 2. 의존성 기반 자동 시작
# 3. 다중 노드 지원
# 4. 컨테이너 격리
```

### 1.3 핵심 개념

**워크로드 (Workload)**
```
워크로드 = 실행하고 싶은 프로그램/서비스

예시:
- zenoh-router (통신 미들웨어)
- carla-spawner (차량 시뮬레이터)
- kuksa-bridge (데이터 브릿지)
```

**컨테이너 (Container)**
```
컨테이너 = 프로그램 + 모든 의존성을 하나의 패키지로

마치 도시락처럼:
┌─────────────────────┐
│ Python 프로그램      │
│ + Python 런타임     │
│ + 필요한 라이브러리  │
│ + 설정 파일         │
└─────────────────────┘
→ 어디서나 동일하게 실행
```

**오케스트레이션 (Orchestration)**
```
오케스트레이션 = 여러 프로그램을 조율하여 관리

마치 교향악단의 지휘자처럼:
- 누가 먼저 시작하고
- 누가 의존하는지
- 죽으면 어떻게 할지
→ 자동으로 관리
```

---

## 2. 시스템 아키텍처

### 2.1 전체 구조도

```
┌─────────────────────────────────────────────────────────────┐
│                    💻 사용자 (User)                          │
│                                                              │
│  ank -k apply manifest.yaml  ← CLI 명령어                   │
└──────────────────────┬──────────────────────────────────────┘
                       │
                    HTTP/gRPC
                       │
┌──────────────────────┴──────────────────────────────────────┐
│                 🧠 Ankaios Server                            │
│                 (중앙 제어 시스템)                            │
│                                                              │
│  역할:                                                       │
│  - Manifest 파싱 및 해석                                     │
│  - 워크로드 상태 추적                                         │
│  - 의존성 그래프 관리                                         │
│  - Agent에게 명령 전달                                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                    gRPC 통신
                       │
        ┌──────────────┴──────────────┐
        │                             │
┌───────┴─────────┐         ┌─────────┴────────┐
│  🤖 Agent #1    │         │  🤖 Agent #2     │
│  (police_agent) │         │  (backup_agent)  │
│                 │         │                  │
│  Node 1         │         │  Node 2          │
└───────┬─────────┘         └─────────┬────────┘
        │                             │
   Podman API                    Podman API
        │                             │
┌───────┴─────────┐         ┌─────────┴────────┐
│  🐳 Podman      │         │  🐳 Podman       │
│  (컨테이너 실행) │         │  (컨테이너 실행)  │
└───────┬─────────┘         └─────────┬────────┘
        │                             │
  ┌─────┼─────┬─────┐          ┌──────┼──────┐
  │     │     │     │          │      │      │
[📦C1] [📦C2] [📦C3] [📦C4]    [📦C5] [📦C6]
zenoh  carla kuksa camera    backup backup
```

### 2.2 각 컴포넌트 상세 설명

#### 2.2.1 Ankaios Server (서버)

**역할**: 시스템의 두뇌 (Brain)

```
입력: ankaios_manifest.yaml
       ↓
┌─────────────────────────────┐
│  1. Manifest 파싱            │
│     - YAML 읽기              │
│     - 워크로드 정의 추출      │
│     - 의존성 그래프 생성      │
├─────────────────────────────┤
│  2. 상태 관리                │
│     - 각 워크로드 상태 추적   │
│     - Agent 상태 모니터링    │
│     - 의존성 충족 여부 확인   │
├─────────────────────────────┤
│  3. 결정 및 명령             │
│     - 시작 순서 결정         │
│     - Agent에 명령 전달      │
│     - 재시작 결정            │
└─────────────────────────────┘
       ↓
출력: Agent에게 gRPC 명령
```

**실행 방법**:
```bash
# 포그라운드 실행 (디버깅용)
ank-server

# 백그라운드 실행
ank-server &

# 로그 확인
tail -f /tmp/ankaios-server.log
```

**포트 및 통신**:
```
기본 포트: 25551 (gRPC)
통신 방식: gRPC with Protobuf
보안: TLS 옵션 지원 (선택적)
```

#### 2.2.2 Ankaios Agent (에이전트)

**역할**: 실제 일꾼 (Worker)

```
Server로부터 명령 수신
       ↓
┌─────────────────────────────┐
│  1. 명령 처리                │
│     - "zenoh-router 시작"    │
│     - "carla-spawner 재시작" │
│     - "상태 보고"            │
├─────────────────────────────┤
│  2. Podman 제어             │
│     - 컨테이너 생성          │
│     - 컨테이너 시작/중지     │
│     - 리소스 모니터링        │
├─────────────────────────────┤
│  3. 상태 보고               │
│     - 컨테이너 상태 확인     │
│     - 에러 발생 시 보고      │
│     - 주기적 상태 업데이트   │
└─────────────────────────────┘
       ↓
Podman을 통해 컨테이너 실행
```

**실행 방법**:
```bash
# 에이전트 시작 (이름 지정 필수)
ank-agent --name police_agent

# 여러 노드에서 실행 가능
# Node 1:
ank-agent --name node1_agent

# Node 2:
ank-agent --name node2_agent

# 로그 확인
tail -f /tmp/ankaios-agent.log
```

**설정 예시**:
```yaml
# manifest에서 Agent 지정
workloads:
  zenoh-router:
    agent: police_agent  # ← 이 Agent에서 실행

  backup-service:
    agent: backup_agent  # ← 다른 Agent에서 실행
```

#### 2.2.3 Podman Runtime

**역할**: 컨테이너 실행 엔진

```
Agent로부터 명령
       ↓
┌─────────────────────────────┐
│  Podman Engine              │
│                             │
│  podman run                 │
│    --network host           │
│    --name zenoh-router      │
│    eclipse/zenoh:latest     │
├─────────────────────────────┤
│  컨테이너 생성 및 실행       │
│  - 이미지 다운로드           │
│  - 네트워크 설정             │
│  - 볼륨 마운트              │
│  - 프로세스 시작             │
└─────────────────────────────┘
       ↓
실행 중인 컨테이너
```

**Podman vs Docker**:
```
┌─────────────────────────────────────────┐
│  Docker                                 │
│  - 데몬 필요 (dockerd)                   │
│  - root 권한 필요                        │
│  - 단일 장애점                          │
└─────────────────────────────────────────┘
              vs
┌─────────────────────────────────────────┐
│  Podman                                 │
│  - 데몬 불필요 (daemonless)              │
│  - rootless 지원                        │
│  - 더 안전함                            │
│  - Kubernetes-compatible                │
└─────────────────────────────────────────┘
```

---

## 3. 작동 원리

### 3.1 시스템 시작 과정 (Step-by-Step)

#### Step 1: 사용자가 `./start_ankaios.sh` 실행

```bash
#!/bin/bash
# start_ankaios.sh 내부

# 1. Prerequisites 체크
check_carla_running()
check_images_exist()
check_ankaios_installed()

# 2. Ankaios Server 시작
ank-server &
# → 백그라운드에서 실행
# → 포트 25551에서 대기

# 3. Ankaios Agent 시작
ank-agent --name police_agent &
# → Server에 자동 연결
# → Podman과 통신 준비

# 4. Manifest 적용
ank -k apply ankaios_manifest.yaml
# → 여기서 마법이 시작됩니다!
```

#### Step 2: Server가 Manifest 파싱

```yaml
# ankaios_manifest.yaml
apiVersion: v0.1
workloads:
  zenoh-router:
    runtime: podman
    agent: police_agent
    restartPolicy: ALWAYS
    dependencies: {}
    runtimeConfig: |
      image: eclipse/zenoh:latest
      commandOptions: ["--network", "host"]
```

**Server 내부 처리**:
```
1. YAML 파싱
   └─ apiVersion 확인: v0.1 ✓
   └─ workloads 섹션 파싱

2. 각 워크로드 분석
   ┌─────────────────────────────┐
   │ zenoh-router                │
   │  - runtime: podman          │
   │  - agent: police_agent      │
   │  - restartPolicy: ALWAYS    │
   │  - dependencies: (없음)     │
   │  - runtimeConfig: 이미지정보 │
   └─────────────────────────────┘

3. 의존성 그래프 생성
   zenoh-router (의존성 없음, 먼저 시작)
       ↓
   carla-spawner (zenoh-router 필요)
       ↓
   ├─ kuksa-bridge (zenoh + carla 필요)
   └─ camera-bridge (zenoh + carla 필요)
       ↓
   ├─ camera-websocket
   └─ police-websocket

4. 시작 순서 결정
   우선순위 큐 생성:
   [1] zenoh-router
   [2] carla-spawner
   [3] kuksa-bridge, camera-bridge (병렬)
   [4] camera-websocket, police-websocket (병렬)
```

#### Step 3: Agent에게 명령 전달

**gRPC 통신 예시**:
```protobuf
// Server → Agent
message StartWorkloadRequest {
  workload_name: "zenoh-router"
  agent_name: "police_agent"
  runtime: "podman"
  runtime_config: {
    image: "eclipse/zenoh:latest"
    command_options: ["--network", "host"]
  }
}
```

**Agent 내부 처리**:
```
1. 명령 수신
   └─ "zenoh-router를 시작하세요"

2. Podman API 호출
   ┌──────────────────────────────────┐
   │ podman run                       │
   │   --network host                 │
   │   --name zenoh-router            │
   │   --restart always               │
   │   eclipse/zenoh:latest           │
   │   --cfg "transport:..."          │
   └──────────────────────────────────┘

3. 결과 확인
   - 컨테이너 ID: abc123def456
   - 상태: Running
   - 포트: (host network 사용)

4. Server에 보고
   └─ "zenoh-router가 성공적으로 시작됨"
```

#### Step 4: 상태 추적 및 다음 워크로드 시작

```
Server 상태 테이블:
┌──────────────────┬─────────────┬──────────┐
│ Workload         │ State       │ Container│
├──────────────────┼─────────────┼──────────┤
│ zenoh-router     │ Running ✓   │ abc123   │
│ carla-spawner    │ Pending     │ -        │
│ kuksa-bridge     │ Pending     │ -        │
└──────────────────┴─────────────┴──────────┘

의존성 확인:
carla-spawner의 dependencies:
  └─ zenoh-router: ADD_COND_RUNNING
  └─ 상태: Running ✓
  └─ → 시작 가능!

Server → Agent:
  "carla-spawner를 시작하세요"

업데이트된 상태:
┌──────────────────┬─────────────┬──────────┐
│ Workload         │ State       │ Container│
├──────────────────┼─────────────┼──────────┤
│ zenoh-router     │ Running ✓   │ abc123   │
│ carla-spawner    │ Running ✓   │ def456   │
│ kuksa-bridge     │ Pending     │ -        │
└──────────────────┴─────────────┴──────────┘

이 과정을 모든 워크로드가 시작될 때까지 반복...
```

### 3.2 실시간 상태 동기화

**주기적 상태 업데이트**:
```
Agent → Server (10초마다)
┌────────────────────────────────────┐
│ Heartbeat + Status Report          │
│                                    │
│ {                                  │
│   "agent": "police_agent",         │
│   "workloads": [                   │
│     {                              │
│       "name": "zenoh-router",      │
│       "state": "Running",          │
│       "container_id": "abc123",    │
│       "cpu": "5%",                 │
│       "memory": "120MB"            │
│     },                             │
│     ...                            │
│   ]                                │
│ }                                  │
└────────────────────────────────────┘
```

---

## 4. 워크로드 생명주기

### 4.1 워크로드 상태 다이어그램

```
                    ank apply
                       ↓
              ┌────────────────┐
              │   Pending      │ ← 시작 대기 중
              │  (대기 상태)    │   (의존성 미충족)
              └────────┬───────┘
                       │ 의존성 충족
                       ↓
              ┌────────────────┐
              │   Starting     │ ← Podman 컨테이너 생성 중
              │  (시작 중)     │
              └────────┬───────┘
                       │
                       ↓
              ┌────────────────┐
              │   Running      │ ← 정상 실행 중
              │  (실행 중) ✓   │
              └────────┬───────┘
                       │
         ┌─────────────┼─────────────┐
         │             │             │
    정상 종료      비정상 종료    사용자 명령
         │             │             │
         ↓             ↓             ↓
  ┌──────────┐  ┌──────────┐  ┌──────────┐
  │Succeeded │  │  Failed  │  │ Stopping │
  │(성공 완료)│  │ (실패)   │  │(중지 중) │
  └──────────┘  └─────┬────┘  └─────┬────┘
                      │             │
                재시작 정책          │
                  확인              │
                      │             ↓
                      │       ┌──────────┐
                      └──────→│ Stopped  │
                              │ (중지됨) │
                              └──────────┘
```

### 4.2 각 상태 상세 설명

#### 📝 Pending (대기)

```
상황: 워크로드가 정의되었지만 아직 시작 안됨

이유:
1. 의존성 미충족
   예: carla-spawner가 zenoh-router를 기다림

2. Agent 없음
   예: 지정된 Agent가 아직 연결 안됨

3. 리소스 부족
   예: 메모리가 부족해서 대기

확인 방법:
$ ank -k get workload kuksa-bridge
Name: kuksa-bridge
State: Pending
Reason: Waiting for dependency 'zenoh-router' to be Running
```

#### 🚀 Starting (시작 중)

```
상황: Podman이 컨테이너를 생성하고 시작하는 중

진행 과정:
1. 이미지 다운로드 (필요시)
   └─ docker.io/library/ros:humble 다운로드 중...

2. 컨테이너 생성
   └─ podman create --name kuksa-bridge ...

3. 볼륨 마운트
   └─ -v /workspace:/workspace:Z

4. 네트워크 설정
   └─ --network host

5. 프로세스 시작
   └─ python3 /workspace/kuksa_zenoh_bridge.py

일반적인 시간: 1-5초
이미지 다운로드 필요시: 30초-5분
```

#### ✅ Running (실행 중)

```
상황: 컨테이너가 정상적으로 실행 중

건강 상태 체크:
1. 프로세스 살아있음
   └─ PID 존재 확인

2. Health Check 통과 (설정된 경우)
   └─ ros2 node list | grep kuksa_zenoh_bridge

3. 포트 리스닝 (해당하는 경우)
   └─ 8081 포트 대기 중

모니터링 지표:
- CPU 사용률: 5-10%
- 메모리: 120-200MB
- 네트워크: 5-50 Mbps
```

#### ❌ Failed (실패)

```
상황: 컨테이너가 에러로 종료됨

일반적인 원인:
1. 코드 에러
   └─ Python exception, segmentation fault

2. 의존성 문제
   └─ zenoh 연결 실패, ROS2 노드 연결 안됨

3. 리소스 부족
   └─ OOM (Out of Memory) killed

4. 설정 오류
   └─ 잘못된 환경 변수, 파일 없음

디버깅:
$ podman logs kuksa-bridge
ERROR: Failed to connect to zenoh router
Traceback (most recent call last):
  File "kuksa_zenoh_bridge.py", line 45
    zenoh_session = zenoh.open()
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ConnectionError: Cannot connect to zenoh

재시작 여부:
restartPolicy: ON_FAILURE → 자동 재시작
restartPolicy: NEVER → 재시작 안함
```

#### 🏁 Succeeded (성공 완료)

```
상황: 컨테이너가 정상적으로 종료됨 (exit code 0)

예시:
- 한 번만 실행하는 작업 (batch job)
- 데이터 마이그레이션
- 백업 작업

restartPolicy 동작:
- ALWAYS: 다시 시작 (무한 루프)
- ON_FAILURE: 시작 안함 (정상 종료)
- NEVER: 시작 안함
```

#### 🛑 Stopped (중지됨)

```
상황: 사용자가 의도적으로 중지함

중지 방법:
$ ank -k delete workload kuksa-bridge

내부 동작:
1. Server → Agent: "kuksa-bridge 중지 요청"
2. Agent → Podman: podman stop kuksa-bridge
3. SIGTERM 신호 전송 (graceful shutdown)
4. 10초 대기
5. 여전히 실행 중이면 SIGKILL (강제 종료)

재시작 방법:
$ ank -k apply ankaios_manifest.yaml
# 또는 manifest에 다시 추가
```

---

## 5. 의존성 관리

### 5.1 의존성 타입

#### ADD_COND_RUNNING (가장 일반적)

```yaml
workloads:
  kuksa-bridge:
    dependencies:
      zenoh-router: ADD_COND_RUNNING  # ← 이 타입
      carla-spawner: ADD_COND_RUNNING
```

**의미**:
```
"zenoh-router가 Running 상태가 되어야만
 kuksa-bridge를 시작할 수 있다"
```

**타임라인**:
```
T=0s   zenoh-router: Pending → Starting
T=2s   zenoh-router: Starting → Running ✓
T=2s   kuksa-bridge: Pending → Starting (의존성 충족!)
T=4s   kuksa-bridge: Starting → Running ✓
```

#### ADD_COND_SUCCEEDED

```yaml
workloads:
  main-app:
    dependencies:
      database-migration: ADD_COND_SUCCEEDED
```

**의미**:
```
"database-migration이 성공적으로 완료되어야
 main-app을 시작할 수 있다"
```

**사용 예시**:
```
1. database-migration 실행
   └─ DB 스키마 업데이트
   └─ 데이터 마이그레이션
   └─ exit 0 (성공)

2. main-app 시작
   └─ 이미 마이그레이션 완료된 DB 사용
```

#### ADD_COND_FAILED

```yaml
workloads:
  fallback-service:
    dependencies:
      primary-service: ADD_COND_FAILED
```

**의미**:
```
"primary-service가 실패했을 때만
 fallback-service를 시작한다"
```

**사용 예시** (Failover):
```
1. primary-service 시작 시도
   └─ 연결 실패 (메인 서버 다운)
   └─ Failed 상태

2. fallback-service 자동 시작
   └─ 백업 서버로 전환
```

### 5.2 복잡한 의존성 예시

#### 예시 1: 순차적 의존성

```yaml
workloads:
  # 1단계: 인프라
  database:
    dependencies: {}

  message-queue:
    dependencies: {}

  # 2단계: 백엔드 서비스
  auth-service:
    dependencies:
      database: ADD_COND_RUNNING

  user-service:
    dependencies:
      database: ADD_COND_RUNNING
      auth-service: ADD_COND_RUNNING

  # 3단계: 프론트엔드
  web-frontend:
    dependencies:
      auth-service: ADD_COND_RUNNING
      user-service: ADD_COND_RUNNING
```

**시작 순서**:
```
시간축 →

T=0s: [database] [message-queue]  ← 동시 시작 (의존성 없음)
       ↓
T=5s: [auth-service]  ← database가 Running
       ↓
T=8s: [user-service]  ← database + auth-service Running
       ↓
T=12s: [web-frontend]  ← auth + user 서비스 Running
```

#### 예시 2: 병렬 의존성

```yaml
workloads:
  shared-cache:
    dependencies: {}

  # 이 셋은 cache만 필요 (서로 독립적)
  api-server-1:
    dependencies:
      shared-cache: ADD_COND_RUNNING

  api-server-2:
    dependencies:
      shared-cache: ADD_COND_RUNNING

  api-server-3:
    dependencies:
      shared-cache: ADD_COND_RUNNING
```

**시작 순서**:
```
T=0s: [shared-cache]
       ↓
T=3s: [api-1] [api-2] [api-3]  ← 동시 시작!
      (서로 의존하지 않으므로 병렬 가능)
```

### 5.3 의존성 체인 디버깅

#### 상황: kuksa-bridge가 시작 안됨

```bash
# 1단계: 상태 확인
$ ank -k get workload kuksa-bridge
Name: kuksa-bridge
State: Pending
Dependencies:
  - zenoh-router: WAITING (Current: Starting)
  - carla-spawner: WAITING (Current: Failed)

# 2단계: 의존성 확인
$ ank -k get workload zenoh-router
State: Starting  # ← 아직 Running이 아님

$ ank -k get workload carla-spawner
State: Failed  # ← 문제 발견!
Error: Cannot connect to CARLA simulator

# 3단계: 문제 해결
# CARLA 시뮬레이터가 실행 중인지 확인
$ nc -z localhost 2000
Connection refused  # ← CARLA가 안 켜져있음!

# CARLA 시작
$ cd /opt/carla && ./CarlaUE4.sh

# 4단계: 재시도
# carla-spawner가 자동으로 재시작됨 (ON_FAILURE)
# → Running 상태
# → kuksa-bridge의 의존성 충족
# → kuksa-bridge 자동 시작!
```

---

## 6. 재시작 정책

### 6.1 ALWAYS (항상 재시작)

```yaml
workloads:
  zenoh-router:
    restartPolicy: ALWAYS
```

**동작 방식**:
```
Container 종료 (어떤 이유든)
    ↓
Ankaios 감지
    ↓
즉시 재시작
    ↓
Running 상태 복구
```

**사용 케이스**:
```
✓ 통신 미들웨어 (zenoh, MQTT broker)
✓ 데이터베이스
✓ 필수 인프라 서비스
✓ "절대 죽으면 안되는" 서비스
```

**예시 시나리오**:
```
T=0s    zenoh-router: Running
T=100s  누군가 실수로 컨테이너 종료
        $ podman stop zenoh-router
T=100s  Ankaios 감지: "zenoh-router가 멈췄다!"
T=101s  Ankaios 자동 재시작
T=103s  zenoh-router: Running (복구 완료)
```

**재시작 횟수 제한**:
```
Ankaios는 무한 재시작을 시도하지만,
백오프(backoff) 전략 사용:

1번째 재시작: 즉시
2번째 재시작: 5초 후
3번째 재시작: 10초 후
4번째 재시작: 20초 후
5번째 이상: 30초 후 (최대)

이유: 계속 실패하는 서비스가
      시스템을 과부하시키지 않도록
```

### 6.2 ON_FAILURE (실패 시만 재시작)

```yaml
workloads:
  kuksa-bridge:
    restartPolicy: ON_FAILURE
```

**동작 방식**:
```
Container 종료
    ↓
Exit Code 확인
    ↓
┌─────────────┬──────────────┐
│ Exit 0      │ Exit != 0    │
│ (정상 종료)  │ (에러 종료)   │
│     ↓       │     ↓        │
│ 재시작 안함  │ 재시작 함     │
└─────────────┴──────────────┘
```

**사용 케이스**:
```
✓ 대부분의 애플리케이션 서비스
✓ 에러는 복구하고 싶지만 정상 종료는 그대로 두고 싶은 경우
✓ 배치 작업 (성공하면 끝, 실패하면 재시도)
```

**예시 시나리오 1 - 에러로 종료**:
```
T=0s    kuksa-bridge: Running
T=50s   Python exception 발생
        Traceback...
        sys.exit(1)  # ← Exit code 1
T=50s   Ankaios 감지: "비정상 종료!"
T=51s   자동 재시작
T=53s   kuksa-bridge: Running (복구)
```

**예시 시나리오 2 - 정상 종료**:
```
T=0s    batch-job: Running
T=30s   작업 완료
        print("Migration complete!")
        sys.exit(0)  # ← Exit code 0
T=30s   Ankaios 감지: "정상 완료"
T=30s   재시작 안함
        State: Succeeded
```

### 6.3 NEVER (재시작 안함)

```yaml
workloads:
  one-time-setup:
    restartPolicy: NEVER
```

**동작 방식**:
```
Container 종료
    ↓
그냥 끝
    ↓
State: Stopped 또는 Failed
```

**사용 케이스**:
```
✓ 초기 설정 작업
✓ 데이터 마이그레이션
✓ 백업 작업
✓ "한 번만 실행하면 되는" 작업
```

### 6.4 재시작 정책 비교표

| 상황 | ALWAYS | ON_FAILURE | NEVER |
|------|--------|------------|-------|
| Exit 0 (정상) | ✅ 재시작 | ❌ 안함 | ❌ 안함 |
| Exit 1 (에러) | ✅ 재시작 | ✅ 재시작 | ❌ 안함 |
| 수동 종료 | ✅ 재시작 | ❌ 안함 | ❌ 안함 |
| OOM Killed | ✅ 재시작 | ✅ 재시작 | ❌ 안함 |

---

## 7. 통신 프로토콜

### 7.1 CLI ↔ Server

**프로토콜**: HTTP/gRPC

```
사용자
  ↓
$ ank -k get workloads
  ↓
┌─────────────────────────┐
│  ank (CLI)              │
│                         │
│  1. 명령어 파싱          │
│  2. HTTP 요청 생성       │
└────────┬────────────────┘
         │
    HTTP GET
         │
         ↓
┌─────────────────────────┐
│  Ankaios Server         │
│                         │
│  1. 요청 처리            │
│  2. 상태 조회            │
│  3. JSON 응답 생성       │
└────────┬────────────────┘
         │
    HTTP Response
         │
         ↓
┌─────────────────────────┐
│  ank (CLI)              │
│                         │
│  1. JSON 파싱            │
│  2. 포맷팅               │
│  3. 터미널 출력          │
└─────────────────────────┘
```

**요청 예시**:
```bash
$ ank -k get workloads

# 내부적으로 변환:
GET http://localhost:25551/api/v1/workloads
Headers:
  Accept: application/json
  Authorization: Bearer <token>  # (인증 활성화 시)
```

**응답 예시**:
```json
{
  "workloads": [
    {
      "name": "zenoh-router",
      "agent": "police_agent",
      "state": "Running",
      "restartPolicy": "ALWAYS",
      "containers": [
        {
          "id": "abc123def456",
          "status": "running",
          "started_at": "2025-10-01T09:00:00Z"
        }
      ]
    },
    ...
  ]
}
```

### 7.2 Server ↔ Agent

**프로토콜**: gRPC (Google Remote Procedure Call)

```
┌──────────────────┐         gRPC          ┌──────────────────┐
│  Ankaios Server  │ ←──────────────────→ │  Ankaios Agent   │
│  (포트: 25551)   │                       │  (클라이언트)     │
└──────────────────┘                       └──────────────────┘

메시지 포맷: Protocol Buffers (protobuf)
특징:
- 바이너리 프로토콜 (빠름)
- 양방향 스트리밍
- 타입 안정성
```

**메시지 예시** (Protobuf 정의):
```protobuf
// 워크로드 시작 요청
message StartWorkloadRequest {
  string workload_name = 1;
  string agent_name = 2;
  string runtime = 3;
  RuntimeConfig runtime_config = 4;
}

// 상태 업데이트
message WorkloadStateUpdate {
  string workload_name = 1;
  WorkloadState state = 2;
  string container_id = 3;
  string error_message = 4;
}

enum WorkloadState {
  PENDING = 0;
  STARTING = 1;
  RUNNING = 2;
  FAILED = 3;
  SUCCEEDED = 4;
}
```

**실제 통신 흐름**:
```
Server → Agent (StartWorkloadRequest)
┌────────────────────────────────────┐
│ workload_name: "kuksa-bridge"      │
│ agent_name: "police_agent"         │
│ runtime: "podman"                  │
│ runtime_config: {                  │
│   image: "police-kuksa-bridge"     │
│   command_options: ["--network"... │
│ }                                  │
└────────────────────────────────────┘
         ↓
Agent → Server (Acknowledgement)
┌────────────────────────────────────┐
│ status: "ACCEPTED"                 │
│ message: "Starting workload..."    │
└────────────────────────────────────┘
         ↓
Agent → Server (StateUpdate - 주기적)
┌────────────────────────────────────┐
│ workload_name: "kuksa-bridge"      │
│ state: RUNNING                     │
│ container_id: "xyz789"             │
└────────────────────────────────────┘
```

### 7.3 Agent ↔ Podman

**프로토콜**: HTTP/REST API (Podman API)

```
┌──────────────────┐    HTTP REST    ┌──────────────────┐
│  Ankaios Agent   │ ───────────────→ │  Podman API      │
│                  │                  │  (Unix Socket)   │
└──────────────────┘                  └──────────────────┘

엔드포인트: unix:///run/user/1000/podman/podman.sock
포맷: JSON
```

**API 호출 예시**:

**1. 컨테이너 생성**:
```bash
POST /v1.40/containers/create?name=kuksa-bridge
Content-Type: application/json

{
  "Image": "localhost/police-kuksa-bridge:latest",
  "HostConfig": {
    "NetworkMode": "host",
    "Binds": [
      "/home/seame/workspace:/workspace:Z"
    ]
  },
  "Env": [
    "ROS_DOMAIN_ID=0",
    "PYTHONUNBUFFERED=1"
  ],
  "Cmd": [
    "python3",
    "/workspace/kuksa_zenoh_bridge.py"
  ]
}

Response:
{
  "Id": "abc123def456...",
  "Warnings": []
}
```

**2. 컨테이너 시작**:
```bash
POST /v1.40/containers/abc123def456/start

Response: 204 No Content
```

**3. 상태 조회**:
```bash
GET /v1.40/containers/abc123def456/json

Response:
{
  "Id": "abc123def456...",
  "State": {
    "Status": "running",
    "Running": true,
    "Pid": 12345,
    "StartedAt": "2025-10-01T09:05:30Z"
  },
  "Name": "kuksa-bridge",
  ...
}
```

---

## 8. 실전 시나리오

### 8.1 정상 시작 시나리오

```
T=0s
User: ./start_ankaios.sh

T=1s
System: Ankaios Server 시작
Log: [INFO] Server listening on :25551

T=2s
System: Ankaios Agent 연결
Log: [INFO] Agent 'police_agent' connected

T=3s
User: ank -k apply ankaios_manifest.yaml
Server: Manifest 파싱 완료, 6개 워크로드 발견

T=4s
Server → Agent: "zenoh-router 시작"
Agent → Podman: podman run eclipse/zenoh:latest
Log: [INFO] Starting workload 'zenoh-router'

T=6s
Podman: 컨테이너 시작 완료
Agent → Server: "zenoh-router Running"
Log: [INFO] Workload 'zenoh-router' is Running

T=7s
Server: zenoh-router Running 확인
Server: carla-spawner 의존성 충족 확인
Server → Agent: "carla-spawner 시작"

T=8s
Agent → Podman: podman run police-carla-spawner
Podman: 컨테이너 시작
CARLA: 차량 스폰 시작...

T=13s
CARLA: Tesla Model 3 스폰 완료
Agent → Server: "carla-spawner Running"
Server: kuksa-bridge, camera-bridge 의존성 충족

T=14s
Server → Agent: "kuksa-bridge 시작" (병렬)
Server → Agent: "camera-bridge 시작" (병렬)
Agent: 두 컨테이너 동시 시작

T=17s
Agent → Server: "kuksa-bridge Running"
Agent → Server: "camera-bridge Running"

T=18s
Server → Agent: "camera-websocket 시작"
Server → Agent: "police-websocket 시작"

T=21s
Agent → Server: 두 WebSocket 서버 모두 Running

T=22s
System: ✅ 모든 워크로드 정상 실행 중!

User: firefox police_dashboard_enhanced.html
Dashboard: 실시간 데이터 수신 시작! 🎉
```

### 8.2 컴포넌트 장애 시나리오

```
T=0s
System: 모든 워크로드 정상 실행 중

T=100s
Event: kuksa-bridge에서 zenoh 연결 끊김
kuksa-bridge: ERROR: Lost connection to zenoh router
kuksa-bridge: sys.exit(1)

T=100.5s
Podman: 컨테이너 종료 감지 (exit code 1)
Agent: 상태 변경 감지
Agent → Server: "kuksa-bridge Failed"

T=101s
Server: 재시작 정책 확인
  └─ restartPolicy: ON_FAILURE ✓
  └─ Exit code: 1 (비정상) ✓
  └─ 결정: 재시작!

Server → Agent: "kuksa-bridge 재시작"
Log: [INFO] Restarting failed workload 'kuksa-bridge'

T=102s
Agent → Podman: podman run police-kuksa-bridge
Podman: 새 컨테이너 생성 및 시작

T=104s
kuksa-bridge: zenoh 연결 성공!
kuksa-bridge: VSS 데이터 수신 시작
Agent → Server: "kuksa-bridge Running"

T=105s
System: ✅ 장애 복구 완료!
Downtime: 5초

Dashboard: 데이터 스트림 재개
User: 장애를 거의 눈치채지 못함
```

### 8.3 의존성 장애 전파 시나리오

```
T=0s
System: 모든 워크로드 정상 실행

T=200s
Event: zenoh-router 크래시!
  (메모리 부족으로 OOM Killed)

T=200.5s
Agent → Server: "zenoh-router Failed"
Server: 재시작 정책 확인 (ALWAYS)

T=201s
Server → Agent: "zenoh-router 즉시 재시작"

T=201s (동시에)
kuksa-bridge: ERROR: zenoh connection lost
camera-bridge: ERROR: zenoh connection lost
camera-websocket: ERROR: Cannot receive frames
police-websocket: ERROR: No data from zenoh

T=202s
kuksa-bridge: 재연결 시도 중...
camera-bridge: 재연결 시도 중...
(재시작은 안됨 - 프로세스 자체는 살아있음)

T=203s
zenoh-router: 재시작 완료
Agent → Server: "zenoh-router Running"

T=204s
kuksa-bridge: ✅ zenoh 재연결 성공
camera-bridge: ✅ zenoh 재연결 성공
camera-websocket: ✅ 데이터 수신 재개
police-websocket: ✅ 데이터 수신 재개

T=205s
System: ✅ 전체 시스템 복구 완료!
Total downtime: 5초

교훈:
- zenoh-router는 ALWAYS 정책이라 즉시 재시작
- 다른 서비스들은 재연결 로직 덕분에 살아남음
- 의존성 계층 구조가 복구를 순차적으로 진행
```

### 8.4 수동 업데이트 시나리오

```
T=0s
User: kuksa-bridge 코드 수정함
  └─ 새 기능 추가: 긴급 알림 필터링

T=1s
User: 컨테이너 이미지 재빌드
$ cd /workspace
$ podman build -t localhost/police-kuksa-bridge:latest \
    -f Dockerfile.kuksa-bridge .
Build: ✅ 성공 (30초 소요)

T=31s
User: 새 이미지로 업데이트
$ ank -k delete workload kuksa-bridge
Agent: 컨테이너 정상 종료 시작
Podman: SIGTERM → SIGKILL (10초 timeout)

T=33s
Agent → Server: "kuksa-bridge Stopped"

T=34s
User: 새 manifest 적용 (또는 자동 재시작)
$ ank -k apply ankaios_manifest.yaml
Server: kuksa-bridge 정의 발견

T=35s
Server: 의존성 확인
  └─ zenoh-router: Running ✓
  └─ carla-spawner: Running ✓
  └─ 시작 가능!

Server → Agent: "kuksa-bridge 시작"
  (새 이미지 사용)

T=37s
Podman: 새 이미지로 컨테이너 생성
kuksa-bridge (new): 시작
kuksa-bridge (new): 새 기능으로 실행 중!

T=38s
Agent → Server: "kuksa-bridge Running"

T=39s
System: ✅ 무중단 업데이트 완료!

User: 로그 확인
$ podman logs kuksa-bridge
[INFO] Starting with emergency alert filtering (NEW!)
[INFO] Connected to zenoh
[INFO] Subscribed to police/vss_data
[INFO] Emergency filter: ENABLED ✓
```

---

## 9. 문제 해결

### 9.1 워크로드가 시작 안됨

#### 증상
```bash
$ ank -k get workloads
NAME            AGENT           STATE
kuksa-bridge    police_agent    Pending
```

#### 진단 단계

**1단계: 의존성 확인**
```bash
$ ank -k get workload kuksa-bridge

Output:
Name: kuksa-bridge
State: Pending
Dependencies:
  - zenoh-router: WAITING (Current: Failed)
  - carla-spawner: OK (Current: Running)

🔍 문제 발견: zenoh-router가 Failed 상태!
```

**2단계: 실패한 워크로드 조사**
```bash
$ ank -k get workload zenoh-router

Output:
Name: zenoh-router
State: Failed
Last Error: Container exited with code 1
Restart Count: 5
Last Restart: 30s ago

🔍 계속 재시작 중이지만 계속 실패
```

**3단계: 로그 확인**
```bash
$ podman logs zenoh-router

Output:
Error: Address already in use (os error 98)
Error: Cannot bind to 0.0.0.0:7447

🔍 포트 충돌 발견!
```

**4단계: 포트 사용 확인**
```bash
$ ss -tlnp | grep 7447

Output:
LISTEN 0 128 0.0.0.0:7447 0.0.0.0:* users:(("zenohd",pid=1234))

🔍 이미 다른 zenohd 프로세스가 실행 중!
```

**5단계: 해결**
```bash
# 기존 프로세스 종료
$ pkill zenohd

# 또는 기존 컨테이너 정리
$ podman stop zenoh-router-old
$ podman rm zenoh-router-old

# Ankaios가 자동으로 재시작
# (restartPolicy: ALWAYS)

# 확인
$ ank -k get workload zenoh-router
State: Running ✓
```

### 9.2 컨테이너가 계속 재시작됨

#### 증상
```bash
$ ank -k get workload carla-spawner

Output:
State: Running
Restart Count: 47  # ← 너무 많음!
Last Restart: 5s ago
```

#### 진단

**1단계: 로그 패턴 분석**
```bash
$ podman logs carla-spawner | tail -50

Output (반복됨):
[INFO] Connecting to CARLA...
[ERROR] Cannot connect to CARLA: Connection refused
[ERROR] Retrying in 5 seconds...
[ERROR] Max retries reached, exiting
Traceback...
sys.exit(1)
```

**2단계: CARLA 상태 확인**
```bash
$ nc -z localhost 2000
Connection refused  # ← CARLA가 안 켜져있음!
```

**3단계: 해결 옵션**

**옵션 A: CARLA 시작**
```bash
$ cd /opt/carla-simulator
$ ./CarlaUE4.sh

# Ankaios가 자동으로 재시도
# → 이번엔 성공!
```

**옵션 B: 재시작 정책 변경 (임시)**
```yaml
# manifest 수정
workloads:
  carla-spawner:
    restartPolicy: NEVER  # 일단 멈춤
```
```bash
$ ank -k apply ankaios_manifest.yaml
# CARLA 준비되면 다시 ALWAYS로 변경
```

### 9.3 Agent가 연결 안됨

#### 증상
```bash
$ ank -k get workloads
Error: No agents connected

$ ank-agent --name police_agent
Error: Cannot connect to server at localhost:25551
```

#### 진단 및 해결

**1단계: Server 실행 확인**
```bash
$ pgrep -a ank-server
(empty)  # ← Server가 안 켜져있음!
```

**해결**:
```bash
$ ank-server &
[INFO] Server started on :25551

$ ank-agent --name police_agent &
[INFO] Connected to server
```

**2단계: 네트워크 확인**
```bash
$ ss -tlnp | grep 25551
LISTEN 0 128 127.0.0.1:25551  # ← localhost만 리스닝

# 다른 머신의 Agent를 연결하려면:
$ ank-server --bind 0.0.0.0:25551  # 모든 인터페이스
```

**3단계: 방화벽 확인**
```bash
$ sudo ufw status
Status: active
To                         Action
--                         ------
25551                      DENY  # ← 차단됨!

# 해결
$ sudo ufw allow 25551/tcp
```

### 9.4 이미지를 못 찾음

#### 증상
```bash
$ ank -k get workload kuksa-bridge
State: Failed
Error: Image localhost/police-kuksa-bridge:latest not found
```

#### 해결
```bash
# 1. 이미지 확인
$ podman images | grep police
(empty)  # ← 이미지가 없음

# 2. 이미지 빌드
$ ./build_containers.sh

# 3. 확인
$ podman images | grep police
localhost/police-base            latest    abc123    2 hours ago    2.1GB
localhost/police-kuksa-bridge    latest    def456    1 hour ago     2.2GB
...

# 4. Ankaios 재시도
$ ank -k apply ankaios_manifest.yaml
# 또는 자동으로 재시작됨 (ON_FAILURE)
```

---

## 10. 고급 활용

### 10.1 다중 노드 배포

**시나리오**: 3대의 컴퓨터로 시스템 분산

```
┌─────────────────────┐
│  Computer 1 (메인)   │
│  - Ankaios Server   │
│  - Agent: main      │
│  - zenoh-router     │
│  - CARLA spawner    │
└─────────────────────┘
          │
    ┌─────┴─────┐
    │           │
┌───┴──────┐ ┌──┴───────┐
│Computer 2│ │Computer 3│
│Agent:    │ │Agent:    │
│ compute1 │ │ compute2 │
│- kuksa   │ │- camera  │
│- bridge  │ │- bridge  │
└──────────┘ └──────────┘
```

**Manifest 설정**:
```yaml
apiVersion: v0.1
workloads:
  # Computer 1에서 실행
  zenoh-router:
    agent: main
    restartPolicy: ALWAYS

  carla-spawner:
    agent: main
    dependencies:
      zenoh-router: ADD_COND_RUNNING

  # Computer 2에서 실행
  kuksa-bridge:
    agent: compute1
    dependencies:
      zenoh-router: ADD_COND_RUNNING
      carla-spawner: ADD_COND_RUNNING

  # Computer 3에서 실행
  camera-bridge:
    agent: compute2
    dependencies:
      zenoh-router: ADD_COND_RUNNING
      carla-spawner: ADD_COND_RUNNING
```

**실행**:
```bash
# Computer 1 (메인)
$ ank-server --bind 0.0.0.0:25551 &
$ ank-agent --name main &

# Computer 2
$ ank-agent --name compute1 --server 192.168.1.100:25551 &

# Computer 3
$ ank-agent --name compute2 --server 192.168.1.100:25551 &

# Computer 1에서 배포
$ ank -k apply ankaios_manifest.yaml
```

### 10.2 리소스 제한

```yaml
workloads:
  carla-spawner:
    runtimeConfig: |
      image: police-carla-spawner:latest
      commandOptions: [
        "--memory", "4g",           # 최대 4GB RAM
        "--memory-swap", "4g",      # 스왑 비활성화
        "--cpus", "2.0",            # 최대 2 CPU
        "--cpu-shares", "1024"      # CPU 우선순위
      ]
```

### 10.3 환경별 설정

**개발 환경**:
```yaml
# dev-manifest.yaml
workloads:
  kuksa-bridge:
    runtimeConfig: |
      image: police-kuksa-bridge:dev
      commandOptions: [
        "-v", "/home/dev/code:/workspace:Z",  # 코드 마운트
        "-e", "LOG_LEVEL=DEBUG"                # 디버그 모드
      ]
```

**프로덕션 환경**:
```yaml
# prod-manifest.yaml
workloads:
  kuksa-bridge:
    runtimeConfig: |
      image: police-kuksa-bridge:v1.2.3  # 태그된 버전
      commandOptions: [
        "--read-only",                    # 읽기 전용 파일시스템
        "-e", "LOG_LEVEL=WARNING",        # 경고만
        "--security-opt", "no-new-privileges"  # 보안 강화
      ]
```

### 10.4 헬스 체크

**Dockerfile에 추가**:
```dockerfile
# Dockerfile.kuksa-bridge
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
  CMD ros2 node list | grep kuksa_zenoh_bridge || exit 1
```

**Ankaios가 자동으로**:
```
30초마다 헬스 체크 실행
  ↓
3번 연속 실패
  ↓
컨테이너를 Failed로 표시
  ↓
재시작 정책에 따라 재시작
```

---

## 📝 요약

### 핵심 개념 정리

1. **Ankaios = 자동차용 가벼운 Kubernetes**
2. **Server = 두뇌, Agent = 일꾼, Podman = 실행 엔진**
3. **Manifest = 원하는 상태를 선언**
4. **의존성 = 자동 순서 관리**
5. **재시작 정책 = 자동 장애 복구**

### 명령어 치트시트

```bash
# 시작
./start_ankaios.sh

# 상태 확인
ank -k get workloads
ank -k get workload <name>

# 로그 확인
podman logs <container-name>
podman logs -f <container-name>

# 재시작
ank -k delete workload <name>  # 자동 재시작됨

# 중지
./stop_ankaios.sh
```

### 문제 해결 플로우

```
문제 발생
  ↓
ank -k get workload <name>  ← 상태 확인
  ↓
의존성 문제? → 의존성 워크로드 확인
  ↓
컨테이너 문제? → podman logs 확인
  ↓
이미지 문제? → podman images 확인
  ↓
해결 → Ankaios가 자동 재시도
```

이제 Ankaios의 모든 것을 이해하셨습니다! 🎉
