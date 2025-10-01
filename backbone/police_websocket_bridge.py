#!/usr/bin/env python3
"""
Police VSS WebSocket Bridge
Subscribes to police VSS data from zenoh and serves it via WebSocket on port 8081
"""

import asyncio
import websockets
import json
import logging
import signal
import zenoh
from typing import Set, Dict, Any, Optional
import time
import queue
import threading

class PoliceWebSocketBridge:
    def __init__(self, websocket_port: int = 8081, district: str = "central"):
        self.websocket_port = websocket_port
        self.district = district
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        self.zenoh_session = None
        self.running = False

        # Message queue for efficient broadcasting
        self.message_queue = queue.Queue(maxsize=100)

        # Cache latest data for new clients
        self.latest_data = {
            'vss_data': None,
            'status': None,
            'emergency': None,
            'fleet_status': None,
            'heartbeat': None
        }

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        self._init_zenoh()

    def _init_zenoh(self):
        try:
            zenoh_config = zenoh.Config()
            try:
                zenoh_config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
                zenoh_config.insert_json5("transport/shared_memory/enabled", "true")
                self.logger.info("🚀 Connecting to SHM-enabled Zenoh router...")
            except Exception:
                self.logger.info("📡 Using peer-to-peer Zenoh connection...")

            self.zenoh_session = zenoh.open(zenoh_config)
            self.logger.info("✅ Connected to Zenoh router successfully")

            # Subscribe to police topics
            self.subscribers = []

            # VSS data for all units in the district
            vss_topic = f"police/{self.district}/vehicle/*/vss"
            self.logger.info(f"🔍 Subscribing to VSS topic: {vss_topic}")
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    vss_topic, self._on_vss_data
                )
            )

            # Status data for all units
            status_topic = f"police/{self.district}/vehicle/*/status"
            self.logger.info(f"🔍 Subscribing to status topic: {status_topic}")
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    status_topic, self._on_status_data
                )
            )

            # Emergency alerts
            emergency_topic = f"police/{self.district}/alerts/emergency/*"
            self.logger.info(f"🔍 Subscribing to emergency topic: {emergency_topic}")
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    emergency_topic, self._on_emergency_alert
                )
            )

            # Fleet status
            fleet_topic = f"police/{self.district}/fleet/status"
            self.logger.info(f"🔍 Subscribing to fleet topic: {fleet_topic}")
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    fleet_topic, self._on_fleet_status
                )
            )

            # Bridge heartbeat
            heartbeat_topic = f"police/{self.district}/bridge/heartbeat"
            self.logger.info(f"🔍 Subscribing to heartbeat topic: {heartbeat_topic}")
            self.subscribers.append(
                self.zenoh_session.declare_subscriber(
                    heartbeat_topic, self._on_heartbeat
                )
            )

            self.logger.info("✅ Police zenoh subscribers initialized")

        except Exception as e:
            self.logger.error(f"Failed to initialize Zenoh: {e}")
            raise

    def _decode_sample_json(self, sample) -> Dict[str, Any]:
        """Decode zenoh sample to JSON"""
        try:
            if hasattr(sample.payload, 'decode'):
                payload_str = sample.payload.decode('utf-8')
            else:
                payload_str = bytes(sample.payload).decode('utf-8')
            return json.loads(payload_str)
        except Exception as e:
            self.logger.error(f"Failed to decode sample: {e}")
            return {}

    def _extract_unit_id_from_key(self, key_expr: str) -> Optional[str]:
        """Extract unit ID from zenoh key expression"""
        try:
            # Expected format: police/central/vehicle/UNIT-001/vss
            parts = str(key_expr).split('/')
            if len(parts) >= 4 and parts[2] == 'vehicle':
                return parts[3]
            return None
        except Exception:
            return None

    def _enqueue_message(self, message: Dict[str, Any]):
        """Add message to broadcast queue"""
        try:
            if self.message_queue.full():
                try:
                    self.message_queue.get_nowait()  # Drop oldest message
                except queue.Empty:
                    pass
            self.message_queue.put_nowait(message)
        except Exception as e:
            self.logger.warning(f"Failed to enqueue message: {e}")

    # Zenoh callback handlers
    def _on_vss_data(self, sample):
        """Handle VSS data from police vehicles"""
        try:
            data = self._decode_sample_json(sample)
            unit_id = self._extract_unit_id_from_key(sample.key_expr)

            self.logger.info(f"📊 Received VSS data for {unit_id}")

            message = {
                'type': 'vss_data',
                'unit_id': unit_id,
                'data': data,
                'timestamp': time.time(),
                'topic': str(sample.key_expr)
            }

            self.latest_data['vss_data'] = message
            if self.connected_clients:
                self._enqueue_message(message)

        except Exception as e:
            self.logger.error(f"Error processing VSS data: {e}")

    def _on_status_data(self, sample):
        """Handle status data from police vehicles"""
        try:
            data = self._decode_sample_json(sample)
            unit_id = self._extract_unit_id_from_key(sample.key_expr)

            self.logger.info(f"📋 Received status data for {unit_id}")

            message = {
                'type': 'status',
                'unit_id': unit_id,
                'data': data,
                'timestamp': time.time(),
                'topic': str(sample.key_expr)
            }

            self.latest_data['status'] = message
            if self.connected_clients:
                self._enqueue_message(message)

        except Exception as e:
            self.logger.error(f"Error processing status data: {e}")

    def _on_emergency_alert(self, sample):
        """Handle emergency alerts"""
        try:
            data = self._decode_sample_json(sample)
            unit_id = self._extract_unit_id_from_key(sample.key_expr)

            self.logger.warning(f"🚨 Emergency alert for {unit_id}")

            message = {
                'type': 'emergency_alert',
                'unit_id': unit_id,
                'data': data,
                'timestamp': time.time(),
                'topic': str(sample.key_expr)
            }

            self.latest_data['emergency'] = message
            if self.connected_clients:
                self._enqueue_message(message)

        except Exception as e:
            self.logger.error(f"Error processing emergency alert: {e}")

    def _on_fleet_status(self, sample):
        """Handle fleet status updates"""
        try:
            data = self._decode_sample_json(sample)

            self.logger.info("📊 Received fleet status update")

            message = {
                'type': 'fleet_status',
                'data': data,
                'timestamp': time.time(),
                'topic': str(sample.key_expr)
            }

            self.latest_data['fleet_status'] = message
            if self.connected_clients:
                self._enqueue_message(message)

        except Exception as e:
            self.logger.error(f"Error processing fleet status: {e}")

    def _on_heartbeat(self, sample):
        """Handle bridge heartbeat"""
        try:
            data = self._decode_sample_json(sample)

            self.logger.debug("💓 Bridge heartbeat received")

            message = {
                'type': 'heartbeat',
                'data': data,
                'timestamp': time.time(),
                'topic': str(sample.key_expr)
            }

            self.latest_data['heartbeat'] = message
            # Don't broadcast heartbeats to save bandwidth

        except Exception as e:
            self.logger.error(f"Error processing heartbeat: {e}")

    # WebSocket handling
    async def _broadcast_message(self, message: Dict[str, Any]):
        """Broadcast message to all connected WebSocket clients"""
        if not self.connected_clients:
            return

        try:
            payload = json.dumps(message)
        except Exception as e:
            self.logger.error(f"Failed to serialize message: {e}")
            return

        disconnected = set()
        for client in list(self.connected_clients):
            try:
                await asyncio.wait_for(client.send(payload), timeout=5.0)
            except Exception:
                disconnected.add(client)

        if disconnected:
            self.connected_clients -= disconnected
            self.logger.info(f"Removed {len(disconnected)} disconnected clients")

    async def _handle_websocket_client(self, websocket):
        """Handle new WebSocket client connection"""
        client_address = "unknown"
        try:
            client_address = websocket.remote_address
            self.logger.info(f"🔗 New WebSocket client connected from {client_address}")
            self.connected_clients.add(websocket)

            # Send latest cached data to new client
            await self._send_initial_data(websocket)

            # Handle client messages
            async for message in websocket:
                try:
                    if isinstance(message, str):
                        data = json.loads(message)
                        await self._handle_client_message(websocket, data)
                except Exception as e:
                    self.logger.warning(f"Error handling client message: {e}")

        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client {client_address} disconnected")
        except Exception as e:
            self.logger.error(f"Error handling client {client_address}: {e}")
        finally:
            self.connected_clients.discard(websocket)
            self.logger.info(f"Client {client_address} cleanup completed")

    async def _send_initial_data(self, websocket):
        """Send cached data to newly connected client"""
        try:
            # Send connection confirmation
            welcome_msg = {
                'type': 'connection_established',
                'district': self.district,
                'timestamp': time.time(),
                'message': f'Connected to police VSS data feed for {self.district} district'
            }
            await websocket.send(json.dumps(welcome_msg))

            # Send latest cached data
            for data_type, data in self.latest_data.items():
                if data is not None:
                    try:
                        await websocket.send(json.dumps(data))
                    except Exception as e:
                        self.logger.warning(f"Failed to send initial {data_type} data: {e}")

        except Exception as e:
            self.logger.error(f"Error sending initial data: {e}")

    async def _handle_client_message(self, websocket, data: Dict[str, Any]):
        """Handle messages from WebSocket clients"""
        try:
            if data.get('type') == 'ping':
                response = {
                    'type': 'pong',
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(response))

            elif data.get('type') == 'status_request':
                response = {
                    'type': 'status_response',
                    'data': {
                        'connected_clients': len(self.connected_clients),
                        'district': self.district,
                        'server_time': time.time(),
                        'latest_data_types': list(self.latest_data.keys())
                    },
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(response))

        except Exception as e:
            self.logger.error(f"Error handling client message: {e}")

    async def _process_message_queue(self):
        """Process message queue and broadcast to clients"""
        self.logger.info("Message queue processor started")

        while self.running:
            try:
                message = self.message_queue.get(timeout=0.1)
                await self._broadcast_message(message)
                self.message_queue.task_done()
            except queue.Empty:
                await asyncio.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error processing message queue: {e}")
                await asyncio.sleep(0.1)

        self.logger.info("Message queue processor stopped")

    async def start_websocket_server(self):
        """Start the WebSocket server"""
        self.logger.info(f"Starting Police WebSocket server on port {self.websocket_port}")
        self.running = True

        # Start message queue processor
        asyncio.create_task(self._process_message_queue())

        # Start WebSocket server
        server = await websockets.serve(
            self._handle_websocket_client,
            "0.0.0.0",
            self.websocket_port,
            ping_interval=20,
            ping_timeout=60,
            max_size=10**6,
            compression=None
        )

        self.logger.info(f"✅ Police WebSocket server started on ws://0.0.0.0:{self.websocket_port}")
        self.logger.info(f"📡 Serving VSS data for district: {self.district}")

        await server.wait_closed()

    def stop(self):
        """Stop the bridge"""
        self.logger.info("Stopping Police WebSocket bridge...")
        self.running = False

        if self.zenoh_session:
            try:
                for subscriber in self.subscribers:
                    subscriber.close()
                self.zenoh_session.close()
            except Exception as e:
                self.logger.warning(f"Error closing zenoh session: {e}")

        self.logger.info("Bridge stopped")

async def main():
    import argparse

    parser = argparse.ArgumentParser(description='Police VSS WebSocket Bridge')
    parser.add_argument('--port', type=int, default=8081, help='WebSocket port (default: 8081)')
    parser.add_argument('--district', type=str, default='central', help='Police district (default: central)')
    args = parser.parse_args()

    bridge = None

    def signal_handler(sig, frame):
        if bridge:
            bridge.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        bridge = PoliceWebSocketBridge(websocket_port=args.port, district=args.district)
        await bridge.start_websocket_server()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Police WebSocket bridge...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if bridge:
            bridge.stop()

if __name__ == "__main__":
    asyncio.run(main())