import threading
import json
import time

# --- MOCK (for local testing) ---
class MockZenoh:
    def __init__(self):
        self._subs = {}

    def subscribe(self, topic, cb):
        self._subs[topic] = cb

    def publish(self, topic, payload):
        # 실제 mock에서는 그냥 pass
        pass

class ZenohInterface:
    def __init__(self, use_mock=True, router_ip="192.168.86.248", port=7447):
        """
        use_mock: True -> 로컬 테스트용 Mock, False -> 실제 Zenoh
        router_ip: Zenoh 라우터 IP
        port: Zenoh 라우터 포트
        """
        self.use_mock = use_mock
        self._mock = MockZenoh() if use_mock else None
        self._session = None
        self._pubs = {}

        if not self.use_mock:
            try:
                import zenoh
            except ImportError:
                raise RuntimeError("pyzenoh not installed, cannot use real Zenoh interface")

            # Zenoh Config 객체 생성
            conf = zenoh.Config()
            # Zenoh client 모드 + 라우터 연결
            conf.insert_json5("mode", '"client"')
            conf.insert_json5("connect/endpoints", f'["tcp/{router_ip}:{port}"]')

            # Zenoh 세션 열기
            self._session = zenoh.open(conf)

    # ---------------- Subscriptions ----------------
    def subscribe_lockon(self, callback):
        """callback(bool)"""
        if self.use_mock:
            self._mock.subscribe("police/lockon", lambda p: callback(bool(p)))
        else:
            self._session.declare_subscriber(
                "police/lockon",
                lambda sample: callback(json.loads(sample.payload.decode()))
            )

    def subscribe_waypoints(self, callback):
        """callback(list of waypoints)"""
        if self.use_mock:
            self._mock.subscribe("police/waypoints", lambda p: callback(p))
        else:
            self._session.declare_subscriber(
                "police/waypoints",
                lambda sample: callback(json.loads(sample.payload.decode()))
            )

    def subscribe_obstacle(self, callback):
        """callback(bool)"""
        if self.use_mock:
            self._mock.subscribe("police/obstacle", lambda p: callback(bool(p)))
        else:
            self._session.declare_subscriber(
                "police/obstacle",
                lambda sample: callback(json.loads(sample.payload.decode()))
            )

    def subscribe_target_vehicle_position(self, callback):
        """
        동민이가 보내는 실시간 차량 위치 구독
        callback(dict): {'x': float, 'y': float, 'z': float, 'vx': float, 'vy': float, 'timestamp': float}
        """
        if self.use_mock:
            self._mock.subscribe("vehicles/target_position", lambda p: callback(p))
        else:
            self._session.declare_subscriber(
                "vehicles/target_position",
                lambda sample: callback(json.loads(sample.payload.decode()))
            )

    # ---------------- Publishers ----------------
    def publish_state(self, state_dict):
        if self.use_mock:
            self._mock.publish("police/state", state_dict)
        else:
            self._session.put("police/state", json.dumps(state_dict))

    def publish_control(self, ctrl_dict):
        if self.use_mock:
            self._mock.publish("police/control", ctrl_dict)
        else:
            self._session.put("police/control", json.dumps(ctrl_dict))

