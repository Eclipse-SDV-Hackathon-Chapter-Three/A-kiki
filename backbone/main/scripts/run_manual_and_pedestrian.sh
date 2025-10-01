#!/usr/bin/env bash
# run_manual_and_pedestrian.sh
# - ./run_manual_vehicle.sh 과 joe_pedestriain.py 를 동시에 실행
# - 로그 분리 저장, Ctrl+C 시 자식 프로세스 정리

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$ROOT_DIR/logs"

MANUAL_SH="$SCRIPT_DIR/run_manual_vehicle.sh"
PEDESTRIAN_PY="$SCRIPT_DIR/joe_pedestriain.py"

mkdir -p "$LOG_DIR"

timestamp() { date +"%Y-%m-%d_%H-%M-%S"; }

log_manual="$LOG_DIR/manual_vehicle_$(timestamp).log"
log_ped="$LOG_DIR/pedestrian_$(timestamp).log"

# 존재/권한 체크
[[ -f "$MANUAL_SH" ]] || { echo "[ERR] not found: $MANUAL_SH"; exit 1; }
[[ -x "$MANUAL_SH" ]] || { echo "[INFO] chmod +x $MANUAL_SH"; chmod +x "$MANUAL_SH"; }
[[ -f "$PEDESTRIAN_PY" ]] || { echo "[ERR] not found: $PEDESTRIAN_PY"; exit 1; }

echo "[INFO] Logs ->"
echo "  manual : $log_manual"
echo "  ped    : $log_ped"
echo

# Ctrl+C 시 자식 프로세스 모두 종료
pids=()
cleanup() {
  echo
  echo "[INFO] Caught signal. Stopping children..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  # 여유 시간 후 강제 종료
  sleep 1
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  echo "[INFO] All stopped."
}
trap cleanup INT TERM

# 1) 수동 차량 실행 (백그라운드)
echo "[START] $MANUAL_SH"
bash -lc "cd '$SCRIPT_DIR' && './$(basename "$MANUAL_SH")'" \
  >"$log_manual" 2>&1 &
pids+=($!)
sleep 0.3

# 2) 보행자 스크립트 실행 (백그라운드)
echo "[START] python3 $(basename "$PEDESTRIAN_PY")"
bash -lc "cd '$SCRIPT_DIR' && python3 '$(basename "$PEDESTRIAN_PY")'" \
  >"$log_ped" 2>&1 &
pids+=($!)

echo
echo "[INFO] All started. Press Ctrl+C to stop."
echo "[INFO] Tailing logs (manual | pedestrian). Press Ctrl+C to exit."

# 두 로그를 동시에 tail
tail -F "$log_manual" "$log_ped" 2>/dev/null &
tail_pid=$!

# 어떤 자식이 먼저 죽으면 알림
wait -n "${pids[@]}" || true
echo "[WARN] One of the processes exited. Check logs above."
kill "$tail_pid" 2>/dev/null || true

# 마무리 정리
cleanup
