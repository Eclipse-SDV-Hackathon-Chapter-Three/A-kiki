#!/usr/bin/env python3
"""
ROS2 Camera to Zenoh Bridge (Optimized Version)
================================================

Subscribes to CARLA compressed camera topics and forwards to zenoh.
Uses pre-compressed images from CARLA for maximum efficiency.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import zenoh
import logging
import argparse
import signal
import sys
from datetime import datetime


class ROS2CameraZenohBridge(Node):
    def __init__(self, zenoh_session):
        super().__init__('ros2_camera_zenoh_bridge')

        self.zenoh_session = zenoh_session

        # Statistics
        self.frame_count = 0
        self.last_report_time = datetime.now()
        self.bytes_published = 0

        # Subscribe to CARLA compressed camera topic
        self.subscriber = self.create_subscription(
            CompressedImage,
            '/carla/hero/camera/image/compressed',
            self.camera_callback,
            10
        )

        # Subscribe to CARLA map camera topic
        self.map_subscriber = self.create_subscription(
            CompressedImage,
            '/carla/map_camera/image/compressed',
            self.map_camera_callback,
            10
        )

        self.get_logger().info('Subscribed to /carla/hero/camera/image/compressed')
        self.get_logger().info('Subscribed to /carla/map_camera/image/compressed')
        self.get_logger().info('Publishing to zenoh: camera/front_rgb/compressed and camera/map/compressed')
        self.get_logger().info('ROS2 Camera Zenoh Bridge started')

    def map_camera_callback(self, msg):
        """Forward compressed map image to zenoh"""
        try:
            # Get compressed JPEG data directly
            jpeg_data = bytes(msg.data)

            # Publish to zenoh
            zenoh_key = "camera/map/compressed"
            self.zenoh_session.put(zenoh_key, jpeg_data)

        except Exception as e:
            self.get_logger().error(f'Error processing map camera frame: {e}')

    def camera_callback(self, msg):
        """Forward compressed image to zenoh"""
        try:
            # Get compressed JPEG data directly
            jpeg_data = bytes(msg.data)

            # Publish to zenoh
            zenoh_key = "camera/front_rgb/compressed"
            self.zenoh_session.put(zenoh_key, jpeg_data)

            # Update statistics
            self.frame_count += 1
            self.bytes_published += len(jpeg_data)

            # Report statistics every 5 seconds
            now = datetime.now()
            elapsed = (now - self.last_report_time).total_seconds()
            if elapsed >= 5.0:
                fps = self.frame_count / elapsed
                mbps = (self.bytes_published * 8 / 1_000_000) / elapsed
                avg_size_kb = (self.bytes_published / self.frame_count) / 1024

                self.get_logger().info(
                    f'Stats: {fps:.1f} fps, {mbps:.2f} Mbps, '
                    f'avg {avg_size_kb:.1f} KB/frame'
                )

                # Reset statistics
                self.frame_count = 0
                self.bytes_published = 0
                self.last_report_time = now

        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {e}')


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\nShutting down camera bridge...')
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='ROS2 Camera to Zenoh Bridge')
    parser.add_argument('--zenoh-mode', type=str, default='peer',
                        choices=['peer', 'client'],
                        help='Zenoh session mode (default: peer)')
    args = parser.parse_args()

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

    # Handle Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize ROS2
    rclpy.init()

    # Initialize Zenoh
    logging.info('Initializing zenoh session...')
    zenoh_config = zenoh.Config()
    if args.zenoh_mode == 'peer':
        zenoh_config.insert_json5('mode', '"peer"')
    else:
        zenoh_config.insert_json5('mode', '"client"')

    zenoh_session = zenoh.open(zenoh_config)
    logging.info(f'Zenoh session opened in {args.zenoh_mode} mode')

    # Create bridge node
    bridge_node = ROS2CameraZenohBridge(zenoh_session)

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        bridge_node.destroy_node()
        rclpy.shutdown()
        zenoh_session.close()
        logging.info('Camera bridge shutdown complete')


if __name__ == '__main__':
    main()
