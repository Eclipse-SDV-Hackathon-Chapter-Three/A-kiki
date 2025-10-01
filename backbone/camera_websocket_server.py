#!/usr/bin/env python3
"""
Camera WebSocket Server
========================

Subscribes to camera images from zenoh and streams them to web clients via WebSocket.

This replaces the C++ zenoh_ws_shm_sub with a simpler Python implementation.
"""

import asyncio
import websockets
import zenoh
import logging
import argparse
import signal
import json
from datetime import datetime
from collections import defaultdict


class CameraWebSocketServer:
    def __init__(self, zenoh_session, port=8080):
        self.zenoh_session = zenoh_session
        self.port = port
        self.clients = set()
        self.camera_subscribers = {}
        self.loop = None  # Will be set in run()

        # Statistics
        self.stats = defaultdict(lambda: {'frames': 0, 'bytes': 0, 'clients': 0})
        self.last_report_time = datetime.now()

        # Subscribe to all camera topics
        self.subscribe_cameras()

        logging.info(f'Camera WebSocket Server initialized on port {port}')

    def subscribe_cameras(self):
        """Subscribe to all camera topics from zenoh"""
        # Subscribe to all camera streams
        camera_selector = 'camera/**/compressed'

        def camera_listener(sample):
            """Handle incoming camera frames from zenoh"""
            try:
                camera_key = sample.key_expr
                jpeg_data = bytes(sample.payload)

                # Extract camera name from key (e.g., "camera/front_rgb/compressed" -> "front_rgb")
                camera_name = str(camera_key).split('/')[1]

                # Update statistics
                self.stats[camera_name]['frames'] += 1
                self.stats[camera_name]['bytes'] += len(jpeg_data)

                # Broadcast to all connected clients
                if self.clients:
                    # Create message with camera metadata
                    message = {
                        'camera': camera_name,
                        'timestamp': datetime.now().isoformat(),
                        'size': len(jpeg_data)
                    }

                    # Send metadata as JSON header + binary JPEG data
                    header = json.dumps(message).encode('utf-8')
                    header_length = len(header).to_bytes(4, byteorder='big')

                    full_message = header_length + header + jpeg_data

                    # Add detailed logging for debugging
                    logging.info(f"Preparing to send frame for '{camera_name}':")
                    logging.info(f"  - Header length: {len(header)}")
                    logging.info(f"  - Header content: {header.decode('utf-8')}")
                    logging.info(f"  - JPEG data (first 20 bytes): {jpeg_data[:20].hex(' ')}")
                    
                    # Broadcast to all clients (non-blocking)
                    if self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast(full_message), self.loop)

            except Exception as e:
                logging.error(f'Error processing camera frame: {e}')

        # Create zenoh subscriber
        subscriber = self.zenoh_session.declare_subscriber(
            camera_selector,
            camera_listener
        )

        self.camera_subscribers['all_cameras'] = subscriber
        logging.info(f'Subscribed to zenoh: {camera_selector}')

    async def broadcast(self, message):
        """Broadcast message to all connected clients"""
        if not self.clients:
            return

        # Send to all clients, remove disconnected ones
        disconnected = set()
        for client in self.clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
            except Exception as e:
                logging.error(f'Error sending to client: {e}')
                disconnected.add(client)

        # Remove disconnected clients
        self.clients -= disconnected

    async def handle_client(self, websocket):
        """Handle new WebSocket client connection"""
        client_addr = websocket.remote_address
        logging.info(f'New client connected: {client_addr}')

        self.clients.add(websocket)

        try:
            # Send welcome message
            welcome = {
                'type': 'welcome',
                'message': 'Connected to camera stream',
                'cameras': list(self.stats.keys()) if self.stats else ['waiting for data']
            }
            await websocket.send(json.dumps(welcome))

            # Keep connection alive and handle client messages
            async for message in websocket:
                # Handle client commands (if any)
                try:
                    cmd = json.loads(message)
                    if cmd.get('type') == 'ping':
                        await websocket.send(json.dumps({'type': 'pong'}))
                except:
                    pass  # Ignore malformed messages

        except websockets.exceptions.ConnectionClosed:
            logging.info(f'Client disconnected: {client_addr}')
        except Exception as e:
            logging.error(f'Error handling client {client_addr}: {e}')
        finally:
            self.clients.discard(websocket)

    async def report_statistics(self):
        """Periodically report server statistics"""
        while True:
            await asyncio.sleep(5.0)

            now = datetime.now()
            elapsed = (now - self.last_report_time).total_seconds()

            if self.stats:
                total_frames = sum(s['frames'] for s in self.stats.values())
                total_bytes = sum(s['bytes'] for s in self.stats.values())
                fps = total_frames / elapsed if elapsed > 0 else 0
                mbps = (total_bytes * 8 / 1_000_000) / elapsed if elapsed > 0 else 0

                logging.info(
                    f'Server stats: {len(self.clients)} clients, '
                    f'{fps:.1f} fps, {mbps:.2f} Mbps'
                )

                # Detailed per-camera stats
                for camera, stat in self.stats.items():
                    cam_fps = stat['frames'] / elapsed if elapsed > 0 else 0
                    cam_mbps = (stat['bytes'] * 8 / 1_000_000) / elapsed if elapsed > 0 else 0
                    logging.debug(
                        f'  {camera}: {cam_fps:.1f} fps, {cam_mbps:.2f} Mbps'
                    )

            # Reset statistics
            self.stats.clear()
            self.last_report_time = now

    async def run(self):
        """Start the WebSocket server"""
        self.loop = asyncio.get_running_loop()

        # Start statistics reporting task
        stats_task = asyncio.create_task(self.report_statistics())

        # Start WebSocket server
        async with websockets.serve(self.handle_client, '0.0.0.0', self.port):
            logging.info(f'Camera WebSocket server running on ws://0.0.0.0:{self.port}')
            await asyncio.Future()  # Run forever


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\nShutting down camera WebSocket server...')
    exit(0)


def main():
    parser = argparse.ArgumentParser(description='Camera WebSocket Server')
    parser.add_argument('--port', type=int, default=8080,
                        help='WebSocket server port (default: 8080)')
    parser.add_argument('--zenoh-mode', type=str, default='peer',
                        choices=['peer', 'client'],
                        help='Zenoh session mode (default: peer)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    args = parser.parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='[%(asctime)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Handle Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize Zenoh
    logging.info('Initializing zenoh session...')
    zenoh_config = zenoh.Config()
    if args.zenoh_mode == 'peer':
        zenoh_config.insert_json5('mode', '"peer"')
    else:
        zenoh_config.insert_json5('mode', '"client"')

    zenoh_session = zenoh.open(zenoh_config)
    logging.info(f'Zenoh session opened in {args.zenoh_mode} mode')

    # Create and run server
    server = CameraWebSocketServer(zenoh_session, port=args.port)

    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        for subscriber in server.camera_subscribers.values():
            subscriber.undeclare()
        zenoh_session.close()
        logging.info('Camera WebSocket server shutdown complete')


if __name__ == '__main__':
    main()
