# publisher_lock_on.py
import argparse
import json
import time

import zenoh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", default="lock_on", help="zenoh key expression (topic)")
    parser.add_argument("--once", action="store_true", help="한 번만 전송하고 종료")
    parser.add_argument("--period", type=float, default=1.0, help="주기(sec)")
    args = parser.parse_args()

    # 세션 오픈
    config = zenoh.Config()
    # 필요 시: config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    session = zenoh.open(config)
    print("[PUB] zenoh session opened")

    # 퍼블리셔 선언
    pub = session.declare_publisher(args.key)
    print(f"[PUB] publishing on '{args.key}'")

    try:
        payload = json.dumps(True).encode("utf-8")  # 그냥 'true' 문자열을 보냄(JSON)
        if args.once:
            pub.put(payload)
            print("[PUB] sent: true (once)")
        else:
            while True:
                pub.put(payload)
                print("[PUB] sent: true")
                time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        pub.undeclare()
        session.close()
        print("[PUB] closed")


if __name__ == "__main__":
    main()
