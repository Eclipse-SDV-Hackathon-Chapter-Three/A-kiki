#!/usr/bin/env python3
"""
zenoh_sniffer.py
- Subscribes to Zenoh keys (default: imu, lidar_gps, target_gps)
- Pretty-prints JSON payloads (or raw text if not JSON)
- Handles graceful shutdown on Ctrl+C
"""

import json
import argparse
import time
from datetime import datetime

# ---- Zenoh ----
try:
    import zenoh
except ImportError:
    raise SystemExit("[ERROR] 'zenoh' 패키지를 찾을 수 없습니다.  pip install zenoh  로 설치하세요.")


def parse_args():
    p = argparse.ArgumentParser(description="Zenoh topic sniffer/pretty-printer")
    p.add_argument(
        "--endpoints",
        default="tcp/127.0.0.1:7447",
        help='Zenoh router endpoints (comma-separated), e.g. "tcp/127.0.0.1:7447,tcp/192.168.0.10:7447"',
    )
    p.add_argument(
        "--keys",
        nargs="*",
        default=["imu", "lidar_gps", "target_gps"],
        help="Zenoh keys to subscribe (space-separated). Use ** to subscribe to all.",
    )
    p.add_argument(
        "--shm",
        action="store_true",
        help="Enable shared-memory transport if available",
    )
    p.add_argument(
        "--raw",
        action="store_true",
        help="Do not pretty-print JSON; show raw payload",
    )
    return p.parse_args()


def _now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _payload_to_bytes(sample):
    # zenoh python exposes sample.payload; in some versions it's bytes, in others it’s a wrapper
    p = getattr(sample, "payload", None)
    if p is None and hasattr(sample, "value"):
        p = getattr(sample.value, "payload", None)
    if p is None:
        return b""
    for attr in ("to_bytes", "get_payload", "get_value"):
        if hasattr(p, attr):
            try:
                return getattr(p, attr)()
            except Exception:
                pass
    try:
        return bytes(p)
    except Exception:
        return b""


def main():
    args = parse_args()

    # ---- Build Zenoh config ----
    cfg = zenoh.Config()
    if args.endpoints:
        # multiple endpoints allowed (comma-separated)
        eps = "[" + ",".join(f'"{ep.strip()}"' for ep in args.endpoints.split(",")) + "]"
        cfg.insert_json5("connect/endpoints", eps)
    if args.shm:
        try:
            cfg.insert_json5("transport/shared_memory/enabled", "true")
        except Exception:
            pass

    print(f"[{_now_str()}] Connecting to Zenoh endpoints: {args.endpoints}  (shm={args.shm})")
    session = zenoh.open(cfg)
    print(f"[{_now_str()}] ✅ Connected to Zenoh")

    subs = []

    def on_sample(sample):
        key = getattr(sample, "key_expr", None) or getattr(sample, "key", None) or "<?>"
        data = _payload_to_bytes(sample)
        ts = _now_str()

        if args.raw:
            try:
                text = data.decode("utf-8", errors="replace")
            except Exception:
                text = str(data)
            print(f"\n[{ts}] [{key}] RAW:\n{text}")
            return

        # Try JSON pretty-print first
        try:
            obj = json.loads(data.decode("utf-8"))
            pretty = json.dumps(obj, ensure_ascii=False, indent=2)
            print(f"\n[{ts}] [{key}] JSON:")
            print(pretty)
        except Exception:
            # Fallback to raw utf-8 text
            try:
                text = data.decode("utf-8", errors="replace")
            except Exception:
                text = str(data)
            print(f"\n[{ts}] [{key}] TEXT:\n{text}")

    # ---- Declare subscribers ----
    keyexprs = args.keys if args.keys else ["**"]
    for k in keyexprs:
        sub = session.declare_subscriber(k, on_sample)
        subs.append(sub)
        print(f"[{_now_str()}] 📡 Subscribed to '{k}'")

    print("\n--- Sniffing... Press Ctrl+C to stop ---\n")

    try:
        # Keep process alive
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print(f"\n[{_now_str()}] Shutting down...")
    finally:
        for s in subs:
            try:
                s.undeclare()
            except Exception:
                pass
        try:
            session.close()
        except Exception:
            pass
        print(f"[{_now_str()}] Bye!")


if __name__ == "__main__":
    main()
