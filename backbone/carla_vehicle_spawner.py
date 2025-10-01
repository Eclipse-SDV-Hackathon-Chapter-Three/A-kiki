#!/usr/bin/env python3
"""
CARLA Vehicle Spawner for ROS2 Bridge
Spawns a vehicle with sensors and publishes data to ROS2 topics
"""

import glob
import os
import sys

# Add CARLA Python API to path
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    try:
        sys.path.append(glob.glob('/home/seame/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

import carla
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import json
import time
import numpy as np
import cv2
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    print("Warning: cv_bridge not available, using basic image conversion")
import logging
import threading
import random
import math
from datetime import datetime, timezone
import uuid

class CarlaVehicleSpawner(Node):
    def __init__(self):
        # Use unique node name with timestamp to avoid conflicts
        import time
        node_name = f'carla_vehicle_spawner_{int(time.time())}'
        super().__init__(node_name)

        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # CARLA connection
        self.client = None
        self.world = None
        self.vehicle = None
        self.sensors = {}

        # ROS2 publishers
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
        self.setup_publishers()

        # VSS data generation
        self.setup_vss_data()

        # Police simulation state
        self.simulation_time = 0
        self.emergency_scenarios = self._initialize_emergency_scenarios()
        self.current_scenario = None
        self.scenario_start_time = None

        # Connect to CARLA
        self.connect_to_carla()

        # Spawn vehicle and sensors
        self.spawn_vehicle()
        self.setup_sensors()

        self.logger.info("CARLA Vehicle Spawner initialized successfully")

    def setup_publishers(self):
        """Setup ROS2 publishers for vehicle data"""
        self.camera_pub = self.create_publisher(CompressedImage, '/carla/hero/camera/image/compressed', 2000)
        self.telemetry_pub = self.create_publisher(String, '/carla/hero/vehicle_status', 10)
        self.odometry_pub = self.create_publisher(PoseStamped, '/carla/hero/odometry', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/carla/hero/lidar/point_cloud', 10)

        # VSS data publishers
        self.vss_publisher = self.create_publisher(String, 'police/vss_data', 10)

        # Timer for telemetry and VSS publishing
        self.timer = self.create_timer(0.1, self.publish_telemetry)  # 10Hz
        self.vss_timer = self.create_timer(0.5, self.publish_vss_data)  # 2Hz for VSS data
        self.emergency_timer = self.create_timer(0.1, self.check_emergency_status)  # 10Hz for emergency checks

    def connect_to_carla(self):
        """Connect to CARLA simulator"""
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.logger.info("Connected to CARLA simulator")
        except Exception as e:
            self.logger.error(f"Failed to connect to CARLA: {e}")
            raise

    def cleanup_existing_vehicles(self):
        """Clean up existing vehicles with 'hero' role"""
        try:
            # Get all vehicles in the world
            vehicles = self.world.get_actors().filter('vehicle.*')
            hero_vehicles = []

            for vehicle in vehicles:
                try:
                    role_name = vehicle.attributes.get('role_name', '')
                    if role_name == 'hero':
                        hero_vehicles.append(vehicle)
                except:
                    continue

            if hero_vehicles:
                self.logger.info(f"Found {len(hero_vehicles)} existing hero vehicles. Cleaning up...")
                for vehicle in hero_vehicles:
                    try:
                        # Also destroy attached sensors
                        sensors = self.world.get_actors().filter('sensor.*')
                        for sensor in sensors:
                            if sensor.parent and sensor.parent.id == vehicle.id:
                                sensor.destroy()

                        vehicle.destroy()
                        self.logger.info(f"Destroyed existing vehicle: {vehicle.id}")
                    except Exception as e:
                        self.logger.warning(f"Failed to destroy vehicle {vehicle.id}: {e}")

                # Wait for cleanup to complete
                time.sleep(1)

        except Exception as e:
            self.logger.warning(f"Error during vehicle cleanup: {e}")

    def spawn_vehicle(self):
        """Spawn a vehicle in CARLA"""
        try:
            # First, clean up any existing hero vehicles
            self.cleanup_existing_vehicles()

            # Get vehicle blueprint
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
            vehicle_bp.set_attribute('role_name', 'hero')

            # Get spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[0] if spawn_points else carla.Transform()

            # Spawn vehicle
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.logger.info(f"Spawned vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")

            # Enable autopilot
            self.vehicle.set_autopilot(True)

        except Exception as e:
            self.logger.error(f"Failed to spawn vehicle: {e}")
            raise

    def setup_sensors(self):
        """Setup sensors on the vehicle"""
        try:
            blueprint_library = self.world.get_blueprint_library()

            # Camera sensor
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '1080')
            camera_bp.set_attribute('image_size_y', '720')
            camera_bp.set_attribute('fov', '90')

            camera_transform = carla.Transform(carla.Location(x=2.0, z=1.5))
            self.sensors['camera'] = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
            self.sensors['camera'].listen(self.camera_callback)

            # LiDAR sensor
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '90000')
            lidar_bp.set_attribute('rotation_frequency', '40')
            lidar_bp.set_attribute('range', '20')

            lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.0))
            self.sensors['lidar'] = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            self.sensors['lidar'].listen(self.lidar_callback)

            self.logger.info("Sensors setup completed")

        except Exception as e:
            self.logger.error(f"Failed to setup sensors: {e}")

    def camera_callback(self, image):
        """Process camera data and publish to ROS2"""
        try:
            # Convert CARLA image to OpenCV format
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # Remove alpha channel
            # Keep BGR format for JPEG encoding (don't convert to RGB)

            # Convert to compressed image message
            ros_image = CompressedImage()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "ego_camera"
            ros_image.format = "jpeg"

            # Compress image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, compressed_data = cv2.imencode('.jpg', array, encode_param)
            ros_image.data = compressed_data.tobytes()

            self.camera_pub.publish(ros_image)

        except Exception as e:
            self.logger.error(f"Error in camera callback: {e}")

    def lidar_callback(self, lidar_data):
        """Process LiDAR data and publish to ROS2"""
        try:
            # Convert CARLA LiDAR data to ROS2 PointCloud2
            # This is a simplified version - full implementation would need proper PointCloud2 conversion
            points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))

            # For now, just log the point count
            self.logger.debug(f"LiDAR points: {len(points)}")

        except Exception as e:
            self.logger.error(f"Error in LiDAR callback: {e}")

    def publish_telemetry(self):
        """Publish vehicle telemetry data"""
        if self.vehicle is None:
            return

        try:
            # Get vehicle data
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            control = self.vehicle.get_control()

            # Create telemetry message
            telemetry_data = {
                'timestamp': time.time(),
                'position': {
                    'x': transform.location.x,
                    'y': transform.location.y,
                    'z': transform.location.z
                },
                'rotation': {
                    'pitch': transform.rotation.pitch,
                    'yaw': transform.rotation.yaw,
                    'roll': transform.rotation.roll
                },
                'velocity': {
                    'x': velocity.x,
                    'y': velocity.y,
                    'z': velocity.z
                },
                'velocity': np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6,  # Convert to km/h
                'control': {
                    'throttle': control.throttle,
                    'steering': control.steer,  # Fixed: steer -> steering for web dashboard
                    'brake': control.brake,
                    'hand_brake': control.hand_brake,
                    'reverse': control.reverse,
                    'gear': 1  # Add gear info for dashboard
                }
            }

            # Publish telemetry
            msg = String()
            msg.data = json.dumps(telemetry_data)
            self.telemetry_pub.publish(msg)

            # Publish odometry
            odom_msg = PoseStamped()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "map"
            odom_msg.pose.position.x = transform.location.x
            odom_msg.pose.position.y = transform.location.y
            odom_msg.pose.position.z = transform.location.z

            self.odometry_pub.publish(odom_msg)

        except Exception as e:
            self.logger.error(f"Error publishing telemetry: {e}")

    def setup_vss_data(self):
        """Initialize VSS data structure based on CARLA vehicle"""
        self.unit_config = {
            "unit_number": "UNIT-001",
            "district": "Central District",
            "precinct": "1st Precinct",
            "vehicle_vin": "1CARLA82633A123456",
            "vehicle_brand": "Tesla",
            "vehicle_model": "Model 3 Police"
        }

        self.officers = {
            "primary": {
                "badge_number": "12345",
                "name": "Officer Johnson",
                "shift_start": datetime.now(timezone.utc).isoformat(),
                "status": "ON_DUTY"
            },
            "secondary": {
                "badge_number": "12346",
                "name": "Officer Smith",
                "status": "ON_DUTY"
            }
        }

        # Police equipment state
        self.police_state = {
            "emergency_active": False,
            "siren_active": False,
            "lightbar_pattern": "OFF",
            "panic_button": False,
            "dash_camera_recording": True,
            "doors_locked": True,
            "availability": "AVAILABLE",
            "active_call": None,
            "last_update": time.time()
        }

    def _initialize_emergency_scenarios(self):
        """Initialize emergency scenarios for police simulation"""
        return [
            {
                "name": "traffic_stop",
                "duration": 300,  # 5 minutes
                "lightbar": "CODE1",
                "siren": False,
                "availability": "BUSY",
                "call_type": "TRAFFIC STOP",
                "priority": "LOW"
            },
            {
                "name": "pursuit",
                "duration": 180,  # 3 minutes
                "lightbar": "CODE3",
                "siren": True,
                "availability": "EMERGENCY",
                "call_type": "VEHICLE PURSUIT",
                "priority": "CRITICAL"
            },
            {
                "name": "domestic_call",
                "duration": 600,  # 10 minutes
                "lightbar": "CODE2",
                "siren": False,
                "availability": "BUSY",
                "call_type": "DOMESTIC DISTURBANCE",
                "priority": "HIGH"
            },
            {
                "name": "routine_patrol",
                "duration": 1800,  # 30 minutes
                "lightbar": "OFF",
                "siren": False,
                "availability": "AVAILABLE",
                "call_type": None,
                "priority": None
            }
        ]

    def carla_to_vss_data(self):
        """Convert CARLA vehicle state to VSS-compliant police data"""
        if self.vehicle is None:
            return None

        try:
            # Get CARLA vehicle data
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            control = self.vehicle.get_control()

            # Calculate speed in km/h
            speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            speed_kmh = speed_ms * 3.6

            # Convert CARLA coordinates to lat/lon (simplified conversion)
            # For a real system, you'd use proper coordinate transformation
            lat = 37.7749 + (transform.location.y / 111320.0)  # Rough conversion
            lon = -122.4194 + (transform.location.x / (111320.0 * np.cos(np.radians(37.7749))))

            # Update simulation time
            self.simulation_time += 0.5  # 0.5 seconds per call (2Hz)

            # Handle emergency scenarios
            self._update_emergency_scenarios()

            # Build VSS data structure
            vss_data = {
                "metadata": {
                    "unit_id": self.unit_config["unit_number"],
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "source": "CARLA_INTEGRATED",
                    "version": "1.0"
                },
                "Vehicle": {
                    "VehicleIdentification": {
                        "VIN": self.unit_config["vehicle_vin"],
                        "Model": self.unit_config["vehicle_model"],
                        "Brand": self.unit_config["vehicle_brand"],
                        "Police": {
                            "UnitNumber": self.unit_config["unit_number"],
                            "District": self.unit_config["district"],
                            "Precinct": self.unit_config["precinct"]
                        }
                    },
                    "CurrentLocation": {
                        "Latitude": lat,
                        "Longitude": lon,
                        "Altitude": float(transform.location.z),
                        "Heading": float(transform.rotation.yaw % 360),
                        "Timestamp": datetime.now(timezone.utc).isoformat()
                    },
                    "Speed": float(speed_kmh),
                    "Police": {
                        "Status": {
                            "Availability": self.police_state["availability"],
                            "LastUpdate": datetime.now(timezone.utc).isoformat(),
                            "Shift": "DAY",
                            "OnDuty": True
                        },
                        "Emergency": {
                            "Lightbar": {
                                "IsActive": self.police_state["emergency_active"],
                                "Pattern": self.police_state["lightbar_pattern"],
                                "Color": "RED_BLUE" if self.police_state["emergency_active"] else "OFF"
                            },
                            "Siren": {
                                "IsActive": self.police_state["siren_active"],
                                "Volume": 85 if self.police_state["siren_active"] else 0,
                                "Type": "YELP" if self.police_state["siren_active"] else "OFF"
                            },
                            "PanicButton": {
                                "IsPressed": self.police_state["panic_button"],
                                "LastActivated": datetime.now(timezone.utc).isoformat() if self.police_state["panic_button"] else None
                            }
                        },
                        "Equipment": {
                            "Radio": {
                                "IsOn": True,
                                "Channel": "DISPATCH_1",
                                "SignalStrength": random.randint(80, 100)
                            },
                            "DashCamera": {
                                "IsRecording": self.police_state["dash_camera_recording"],
                                "StorageUsed": random.randint(10, 90),
                                "Resolution": "1080p"
                            },
                            "MDT": {
                                "IsConnected": True,
                                "ActiveCall": self.police_state["active_call"],
                                "MessagesUnread": random.randint(0, 5)
                            }
                        },
                        "Officer": {
                            "Primary": self.officers["primary"],
                            "Secondary": self.officers["secondary"]
                        }
                    },
                    "Body": {
                        "Hood": {"IsOpen": False},
                        "Trunk": {"IsOpen": False},
                        "Doors": {
                            "Row1": {
                                "Left": {"IsOpen": False, "IsLocked": self.police_state["doors_locked"]},
                                "Right": {"IsOpen": False, "IsLocked": self.police_state["doors_locked"]}
                            }
                        }
                    },
                    "Powertrain": {
                        "CombustionEngine": {
                            "IsRunning": True,
                            "Speed": float(speed_kmh * 50),  # Simulated RPM
                            "TPS": float(control.throttle * 100)  # Throttle position
                        },
                        "FuelSystem": {
                            "Level": random.uniform(20, 95),  # Fuel percentage
                            "Range": random.randint(200, 500)  # Range in km
                        }
                    },
                    "Service": {
                        "DistanceToService": random.randint(1000, 5000),
                        "TimeToService": random.randint(30, 180),
                        "IsServiceDue": False
                    },
                    "OBD": {
                        "Speed": float(speed_kmh),
                        "EngineLoad": float(control.throttle * 100),
                        "ThrottlePosition": float(control.throttle * 100),
                        "RPM": float(speed_kmh * 50)
                    }
                }
            }

            return vss_data

        except Exception as e:
            self.logger.error(f"Error converting CARLA data to VSS: {e}")
            return None

    def _update_emergency_scenarios(self):
        """Update emergency scenarios based on random events and time"""
        current_time = time.time()

        # Check if current scenario should end
        if self.current_scenario and self.scenario_start_time:
            elapsed = current_time - self.scenario_start_time
            if elapsed >= self.current_scenario["duration"]:
                self.logger.info(f"Emergency scenario '{self.current_scenario['name']}' ended")
                self.current_scenario = None
                self.scenario_start_time = None
                # Reset to routine patrol
                self._apply_scenario(self.emergency_scenarios[-1])  # routine_patrol

        # Random chance to start new emergency scenario (1% per check = ~20% per minute)
        if not self.current_scenario and random.random() < 0.01:
            # Pick random emergency scenario (exclude routine patrol)
            scenario = random.choice(self.emergency_scenarios[:-1])
            self.logger.info(f"Starting emergency scenario: {scenario['name']}")
            self.current_scenario = scenario
            self.scenario_start_time = current_time
            self._apply_scenario(scenario)

        # Panic button simulation (very rare - 0.001% chance)
        if random.random() < 0.00001:
            self.logger.warning("🚨 PANIC BUTTON ACTIVATED!")
            self.police_state["panic_button"] = True
            # Auto-reset after 5 seconds
            threading.Timer(5.0, lambda: setattr(self.police_state, "panic_button", False)).start()

    def _apply_scenario(self, scenario):
        """Apply emergency scenario to police state"""
        self.police_state["availability"] = scenario["availability"]
        self.police_state["lightbar_pattern"] = scenario["lightbar"]
        self.police_state["siren_active"] = scenario["siren"]
        self.police_state["emergency_active"] = scenario["lightbar"] != "OFF"

        if scenario["call_type"]:
            self.police_state["active_call"] = {
                "type": scenario["call_type"],
                "priority": scenario["priority"],
                "start_time": datetime.now(timezone.utc).isoformat(),
                "call_id": str(uuid.uuid4())[:8]
            }
        else:
            self.police_state["active_call"] = None

    def publish_vss_data(self):
        """Publish VSS data generated from CARLA vehicle"""
        vss_data = self.carla_to_vss_data()
        if vss_data:
            try:
                # Publish VSS data
                vss_msg = String()
                vss_msg.data = json.dumps(vss_data)
                self.vss_publisher.publish(vss_msg)

                self.logger.info(f"Published VSS data for {self.unit_config['unit_number']}")

            except Exception as e:
                self.logger.error(f"Error publishing VSS data: {e}")

    def check_emergency_status(self):
        """Check and handle emergency status updates"""
        # This method is called at 10Hz for responsive emergency handling
        # Most of the work is done in _update_emergency_scenarios() called from publish_vss_data()
        pass

    def cleanup(self):
        """Clean up CARLA actors"""
        try:
            if self.sensors:
                for sensor in self.sensors.values():
                    if sensor is not None:
                        sensor.destroy()

            if self.vehicle is not None:
                self.vehicle.destroy()

            self.logger.info("CARLA actors cleaned up")

        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)

    try:
        spawner = CarlaVehicleSpawner()
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'spawner' in locals():
            spawner.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()