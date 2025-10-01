#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_police_all.py — 오케스트레이터 (경찰 시스템은 외부에서 별도 실행)
동작:
  - (사용자가 외부에서) ./run_enhanced_police_system.py 실행/스폰 완료
  - 본 스크립트가 CARLA에서 'hero' 차량 등장까지 대기
  - auto_chase_detecter.py → auto_chase_lidar.py 순서로 실행
  - 어느 하나라도 종료되면 전체 안전 종료
  - detecter가 너무 빨리 종료(code 0)하면 재시도(기본 2회)
"""

import os
import sys
import time
import signal
import subprocess
from pathlib import Path

# ---------- 경로 ----------
BASE = Path("~/Carla_final/Carla_final").expanduser().resolve()
SCRIPT_DETECTER = BASE / "main" / "chase" / "control" / "auto_chase_detecter.py"
SCRIPT_LIDAR    = BASE / "main" / "scripts" / "auto_chase_lidar.py"

LOG_DIR = BASE / "logs"
LOG_DIR.mkdir(exist_ok=True)
LOG_DETECTER = LOG_DIR / "auto_chase_detecter.log"
LOG_LIDAR    = LOG_DIR / "auto_chase_lidar.log"

# ---------- 환경/타이밍 ----------
CARLA_HOST = os.environ.get("CARLA_HOST", "localhost")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))

NICENESS   = int(os.environ.get("RUN_ALL_NICENESS", "5"))
POLL_INTERVAL = float(os.environ.get("POLL_INTERVAL", "0.5"))

WAIT_HERO_TIMEOUT = float(os.environ.get("WAIT_HERO_TIMEOUT", "40.0"))     # hero 대기 최대 시간
DETECTER_EARLY_WINDOW = float(os.environ.get("DETECTER_EARLY_WINDOW", "10.0"))  # 조기종료 판정
DETECTER_MAX_RETRY = int(os.environ.get("DETECTER_MAX_RETRY", "2"))

PROCS = []  # [(Popen, log_f, desc)]

def _preexec():
    """자식에서만: 프로세스 그룹 생성 + niceness 조정"""
    try:
        os.setsid()
    except Exception:
        pass
    try:
        if NICENESS:
            os.nice(NICENESS)
    except Exception:
        pass

def _spawn(cmd, cwd: Path, log_path: Path, desc: str):
    env = os.environ.copy()
    log_f = open(log_path, "a", buffering=1)
    p = subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=log_f,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=_preexec
    )
    PROCS.append((p, log_f, desc))
    print(f"[orchestrator] started {desc} (pid={p.pid})  cwd={cwd}  cmd={' '.join(cmd)}")
    print(f"[orchestrator] logging -> {log_path}")
    return p

def _kill_all(sig=signal.SIGTERM):
    """모든 자식 프로세스를 프로세스 그룹 단위로 종료"""
    for p, _, desc in reversed(PROCS):
        try:
            if p.poll() is None:
                print(f"[orchestrator] sending {sig.name} to {desc} (pid={p.pid})")
                os.killpg(os.getpgid(p.pid), sig)
        except ProcessLookupError:
            pass
        except Exception as e:
            print(f"[orchestrator] kill error for {desc}: {e}")

def _hard_kill_leftovers():
    """남은 프로세스가 있으면 SIGKILL"""
    for p, _, desc in reversed(PROCS):
        try:
            if p.poll() is None:
                print(f"[orchestrator] sending SIGKILL to {desc} (pid={p.pid})")
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

def _close_logs():
    for _, log_f, _ in PROCS:
        try:
            log_f.flush()
            log_f.close()
        except Exception:
            pass

def _validate_paths():
    errors = []
    if not SCRIPT_DETECTER.exists(): errors.append(f"missing: {SCRIPT_DETECTER}")
    if not SCRIPT_LIDAR.exists():    errors.append(f"missing: {SCRIPT_LIDAR}")
    if errors:
        raise FileNotFoundError("\n".join(errors))

def _signal_handler(signum, frame):
    print(f"\n[orchestrator] received signal {signum}, stopping all...")
    _kill_all(signal.SIGTERM)
    time.sleep(1.0)
    _hard_kill_leftovers()
    _close_logs()
    sys.exit(0)

def _carla_has_hero() -> bool:
    """CARLA에 role_name=hero 차량 존재 여부"""
    try:
        import carla
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(2.0)
        world = client.get_world()
        for a in world.get_actors().filter("vehicle.*"):
            try:
                if a.attributes.get("role_name", "").lower() == "hero":
                    return True
            except Exception:
                pass
        return False
    except Exception:
        return False

def _wait_for_hero(timeout_s: float) -> bool:
    print(f"[orchestrator] waiting for hero vehicle from external spawner (timeout {timeout_s:.1f}s)...")
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if _carla_has_hero():
            print("[orchestrator] hero vehicle detected.")
            return True
        time.sleep(POLL_INTERVAL)
    print("[orchestrator] hero vehicle not detected within timeout.")
    return False

def main():
    _validate_paths()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        # 외부 스포너가 hero를 띄웠다고 가정하고 대기
        _wait_for_hero(WAIT_HERO_TIMEOUT)

        detecter_attempt = 0
        while True:
            start_t = time.time()
            _spawn(["python3", str(SCRIPT_DETECTER)], SCRIPT_DETECTER.parent, LOG_DETECTER, "auto_chase_detecter")
            _spawn(["python3", str(SCRIPT_LIDAR)],    SCRIPT_LIDAR.parent,    LOG_LIDAR,    "auto_chase_lidar")

            # 모니터링 루프
            while True:
                for p, _, desc in PROCS:
                    ret = p.poll()
                    if ret is not None:
                        alive = time.time() - start_t

                        # detecter가 너무 빨리 정상 종료 → 재시작 (lidar 포함)
                        if desc == "auto_chase_detecter" and ret == 0 and alive < DETECTER_EARLY_WINDOW and detecter_attempt < DETECTER_MAX_RETRY:
                            detecter_attempt += 1
                            print(f"[orchestrator] detecter early-exit(code=0, {alive:.1f}s). retry {detecter_attempt}/{DETECTER_MAX_RETRY} ...")

                            # 두 자식 종료
                            _kill_all(signal.SIGTERM)
                            time.sleep(1.0)
                            _hard_kill_leftovers()

                            # 프로세스 목록 비우기 (재할당 금지, 슬라이스로 비우기)
                            PROCS[:] = []

                            # hero 재확인(짧게)
                            _wait_for_hero(max(5.0, POLL_INTERVAL*4))
                            break  # 내부 while 탈출 → 외부 while이 다시 스폰
                        # 그 외 종료: 전체 정리 후 종료
                        print(f"[orchestrator] '{desc}' exited with code {ret}. Stopping all...")
                        _kill_all(signal.SIGTERM)
                        time.sleep(1.0)
                        _hard_kill_leftovers()
                        _close_logs()
                        return
                else:
                    time.sleep(POLL_INTERVAL)
                    continue
                # 재시작 케이스: 내부 while에서 break → 외부 while로
                break

    except KeyboardInterrupt:
        print("\n[orchestrator] keyboard interrupt -> stopping...")
    except Exception as e:
        print(f"[orchestrator] error: {e}")
    finally:
        _kill_all(signal.SIGTERM)
        time.sleep(1.0)
        _hard_kill_leftovers()
        _close_logs()
        print("[orchestrator] stopped cleanly.")

if __name__ == "__main__":
    main()
