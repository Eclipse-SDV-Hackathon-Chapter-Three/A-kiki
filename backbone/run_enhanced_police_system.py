#!/usr/bin/env python3
"""
Enhanced Police Monitoring System Launcher
==========================================

Launches the complete enhanced police vehicle monitoring system with:
- VSS-compliant police vehicle data generation
- KUKSA-zenoh integration bridge
- Enhanced police station dashboard
- High-performance SHM camera streaming

This replaces the previous simple JSON telemetry with rich VSS police data.
"""

import os
import sys
import time
import subprocess
import signal
import logging
import json
import webbrowser
from datetime import datetime
from pathlib import Path

class EnhancedPoliceSystemLauncher:
    def __init__(self):
        self.processes = []
        self.config = self.load_config()
        self.setup_logging()
        self.logger = logging.getLogger(__name__)

    def load_config(self):
        """Load system configuration"""
        default_config = {
            "camera_shm_capacity": 67108864,  # 64MB
            "police_district": "central",
            "enable_vss_validation": True,
            "enable_emergency_alerts": True,
            "carla_host": "localhost",
            "carla_port": 2000,
            "police_units": [
                {
                    "unit_id": "UNIT-001",
                    "officer_primary": {
                        "name": "Officer Johnson",
                        "badge": "12345"
                    },
                    "officer_secondary": {
                        "name": "Officer Smith",
                        "badge": "12346"
                    }
                }
            ],
            "dashboard_auto_open": True,
            "log_level": "INFO"
        }

        config_file = Path("config/police_system_config.json")
        if config_file.exists():
            with open(config_file, 'r') as f:
                user_config = json.load(f)
                default_config.update(user_config)

        return default_config

    def setup_logging(self):
        """Setup logging configuration"""
        log_level = getattr(logging, self.config.get('log_level', 'INFO'))
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler(f'enhanced_police_system_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
            ]
        )

    def check_dependencies(self):
        """Check if all required dependencies are available"""
        self.logger.info("🔍 Checking system dependencies...")

        dependencies = {
            "CARLA": self.check_carla_connection,
            "ROS2": self.check_ros2_environment,
            "zenoh": self.check_zenoh_installation,
            "Python packages": self.check_python_packages,
            "C++ binaries": self.check_cpp_binaries
        }

        all_good = True
        for name, check_func in dependencies.items():
            try:
                if check_func():
                    self.logger.info(f"✅ {name}: OK")
                else:
                    self.logger.error(f"❌ {name}: FAILED")
                    all_good = False
            except Exception as e:
                self.logger.error(f"❌ {name}: ERROR - {e}")
                all_good = False

        if not all_good:
            self.logger.error("❌ Dependency check failed. Please fix the issues above.")
            return False

        self.logger.info("✅ All dependencies check passed!")
        return True

    def check_carla_connection(self):
        """Check CARLA simulator connection"""
        try:
            import carla
            client = carla.Client(self.config['carla_host'], self.config['carla_port'])
            client.set_timeout(5.0)
            world = client.get_world()
            return True
        except Exception as e:
            self.logger.warning(f"CARLA not available: {e}")
            return False

    def check_ros2_environment(self):
        """Check ROS2 environment"""
        return os.environ.get('ROS_DISTRO') is not None

    def check_zenoh_installation(self):
        """Check zenoh installation"""
        try:
            # Check zenohd (daemon) first
            result = subprocess.run(['zenohd', '--version'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return True

            # Fallback: check zenoh CLI
            result = subprocess.run(['zenoh', 'info'], capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except:
            return False

    def check_python_packages(self):
        """Check required Python packages"""
        required_packages = ['rclpy', 'zenoh', 'jsonschema']
        for package in required_packages:
            try:
                __import__(package)
            except ImportError:
                self.logger.error(f"Missing Python package: {package}")
                return False
        return True

    def check_cpp_binaries(self):
        """Check if C++ binaries are built - DEPRECATED, now using Python bridges"""
        # No longer needed - using Python bridges instead
        return True

    def start_zenoh_router(self):
        """Start zenoh router with SHM enabled"""
        self.logger.info("🚀 Starting zenoh router with SHM support...")

        try:
            # Check if zenoh router is already running
            result = subprocess.run(['pgrep', '-f', 'zenohd'], capture_output=True)
            if result.returncode == 0:
                self.logger.info("✅ zenoh router already running")
                return True

            # Start zenoh router
            cmd = [
                'zenohd',
                '--cfg', 'transport: {shared_memory: {enabled: true}}'
            ]

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.processes.append(('zenoh_router', process))

            # Wait for router to start
            time.sleep(3)

            if process.poll() is None:
                self.logger.info("✅ zenoh router started successfully")
                return True
            else:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"zenoh router failed to start: {error_msg}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start zenoh router: {e}")
            return False

    def start_carla_vehicle_spawner(self):
        """Start CARLA police vehicle spawner"""
        self.logger.info("🚗 Starting CARLA police vehicle spawner...")

        try:
            # Use enhanced spawner if available, otherwise use regular spawner
            spawner_script = "enhanced_carla_spawner.py"
            if not Path(spawner_script).exists():
                spawner_script = "carla_vehicle_spawner.py"

            process = subprocess.Popen(
                ['python3', spawner_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.processes.append(('carla_spawner', process))

            # Wait for spawner to initialize
            time.sleep(5)

            if process.poll() is None:
                self.logger.info("✅ CARLA police vehicle spawner started")
                return True
            else:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"CARLA spawner failed: {error_msg}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start CARLA spawner: {e}")
            return False

    def start_integrated_carla_vss(self):
        """Start integrated CARLA vehicle spawner with VSS data generation"""
        self.logger.info("🚗 Starting integrated CARLA vehicle spawner with VSS data...")

        try:
            # Set ROS2 environment
            env = os.environ.copy()
            env.update({
                'ROS_DOMAIN_ID': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'
            })

            process = subprocess.Popen(
                [sys.executable, 'carla_vehicle_spawner.py'],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            self.processes.append(('carla_vss_spawner', process))

            # Wait for spawner to initialize
            time.sleep(1)  # Slightly longer for CARLA connection

            if process.poll() is None:
                self.logger.info("✅ Integrated CARLA vehicle spawner with VSS started")
                return True
            else:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"CARLA VSS spawner failed: {error_msg}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start CARLA VSS spawner: {e}")
            return False

    def start_kuksa_zenoh_bridge(self):
        """Start KUKSA-zenoh bridge"""
        self.logger.info("🌉 Starting KUKSA-zenoh bridge...")

        try:
            # Set ROS2 environment and bridge parameters
            env = os.environ.copy()
            env.update({
                'ROS_DOMAIN_ID': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'
            })

            cmd = [
                sys.executable, 'kuksa_zenoh_bridge.py',
                '--ros-args',
                '-p', f'police_district:={self.config["police_district"]}',
                '-p', f'enable_data_validation:={str(self.config["enable_vss_validation"]).lower()}',
                '-p', f'emergency_alert_topics:={str(self.config["enable_emergency_alerts"]).lower()}'
            ]

            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            self.processes.append(('kuksa_bridge', process))

            # Wait for bridge to initialize
            time.sleep(0.5)

            if process.poll() is None:
                self.logger.info("✅ KUKSA-zenoh bridge started")
                return True
            else:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"KUKSA bridge failed: {error_msg}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start KUKSA bridge: {e}")
            return False

    def start_camera_zenoh_bridge(self):
        """Start Python camera zenoh bridge (ROS2 -> zenoh with JPEG compression)"""
        self.logger.info("📸 Starting camera zenoh bridge...")

        try:
            # Set ROS2 environment
            env = os.environ.copy()
            env.update({
                'ROS_DOMAIN_ID': '0',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'
            })

            cmd = [
                sys.executable, 'ros2_camera_zenoh_bridge.py',
                '--zenoh-mode', 'peer'
            ]

            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.processes.append(('camera_zenoh_bridge', process))

            # Wait for bridge to initialize
            time.sleep(0.5)

            if process.poll() is None:
                self.logger.info(f"✅ Camera zenoh bridge started (using CARLA compressed images)")
                return True
            else:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"Camera zenoh bridge failed: {error_msg}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start camera zenoh bridge: {e}")
            return False

    def start_websocket_bridges(self):
        """Start WebSocket bridges for camera and telemetry"""
        self.logger.info("🌐 Starting WebSocket bridges...")

        success = True

        # Start camera WebSocket server (Python)
        try:
            cmd = [
                sys.executable, 'camera_websocket_server.py',
                '--port', '8080',
                '--zenoh-mode', 'peer'
            ]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(('camera_websocket', process))

            time.sleep(0.8)
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"Camera WebSocket server failed: {error_msg}")
                success = False
            else:
                self.logger.info("✅ Camera WebSocket server started (port 8080)")

        except Exception as e:
            self.logger.error(f"Failed to start camera WebSocket server: {e}")
            success = False

        # Start police VSS WebSocket bridge (Python)
        try:
            cmd = [
                sys.executable, 'police_websocket_bridge.py',
                '--port', '8081',
                '--district', self.config["police_district"]
            ]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(('police_websocket', process))

            time.sleep(0.5)
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                error_msg = stderr.decode() if stderr else "Unknown error"
                self.logger.error(f"Police WebSocket bridge failed: {error_msg}")
                success = False
            else:
                self.logger.info("✅ Police VSS WebSocket bridge started (port 8081)")

        except Exception as e:
            self.logger.error(f"Failed to start police WebSocket bridge: {e}")
            success = False

        return success

    def open_dashboard(self):
        """Open the enhanced police dashboard"""
        if not self.config.get('dashboard_auto_open', True):
            return

        self.logger.info("🖥️ Opening enhanced police dashboard...")

        dashboard_path = Path("police_dashboard_enhanced.html").resolve()
        if dashboard_path.exists():
            dashboard_url = f"file://{dashboard_path}"
            try:
                webbrowser.open(dashboard_url)
                self.logger.info(f"✅ Dashboard opened: {dashboard_url}")
            except Exception as e:
                self.logger.warning(f"Could not auto-open dashboard: {e}")
                self.logger.info(f"Please open manually: {dashboard_url}")
        else:
            self.logger.error("Dashboard file not found!")

    def cleanup_processes(self):
        """Clean up all started processes"""
        self.logger.info("🧹 Cleaning up processes...")

        for name, process in self.processes:
            try:
                if process.poll() is None:
                    self.logger.info(f"Terminating {name}...")

                    # Try graceful termination first
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    time.sleep(1)

                    # Force kill if still running
                    if process.poll() is None:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        time.sleep(1)

                    self.logger.info(f"✅ {name} terminated")
                else:
                    self.logger.info(f"✅ {name} already stopped")

            except Exception as e:
                self.logger.error(f"Error terminating {name}: {e}")

        self.processes.clear()

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info("🛑 Shutdown signal received...")
        self.cleanup_processes()
        sys.exit(0)

    def run(self):
        """Run the complete enhanced police system"""
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.logger.info("🚔 Starting Enhanced Police Monitoring System...")
        self.logger.info(f"District: {self.config['police_district']}")
        self.logger.info(f"VSS Validation: {'Enabled' if self.config['enable_vss_validation'] else 'Disabled'}")
        self.logger.info(f"Emergency Alerts: {'Enabled' if self.config['enable_emergency_alerts'] else 'Disabled'}")

        try:
            # Check dependencies
            if not self.check_dependencies():
                self.logger.error("❌ System startup aborted due to missing dependencies")
                return False

            # Start all components
            startup_steps = [
                ("zenoh Router", self.start_zenoh_router),
                ("Integrated CARLA+VSS System", self.start_integrated_carla_vss),
                ("KUKSA-zenoh Bridge", self.start_kuksa_zenoh_bridge),
                ("Camera Zenoh Bridge", self.start_camera_zenoh_bridge),
                ("WebSocket Bridges", self.start_websocket_bridges),
            ]

            for step_name, step_func in startup_steps:
                self.logger.info(f"🔄 Starting {step_name}...")
                if not step_func():
                    self.logger.error(f"❌ Failed to start {step_name}")
                    return False
                time.sleep(1)  # Increased pause between components

            # Open dashboard
            time.sleep(2)  # Wait for all services to stabilize
            self.open_dashboard()

            # System is ready
            self.logger.info("🎉 Enhanced Police Monitoring System started successfully!")
            self.logger.info("📊 System Status:")
            self.logger.info(f"   • VSS Police Data: Publishing rich vehicle data")
            self.logger.info(f"   • KUKSA Integration: Active with validation")
            self.logger.info(f"   • Camera Stream: Using CARLA compressed images")
            self.logger.info(f"   • Emergency Alerts: Real-time monitoring active")
            self.logger.info(f"   • Police District: {self.config['police_district']}")
            self.logger.info("🌐 Access Points:")
            self.logger.info("   • Enhanced Dashboard: file://police_dashboard_enhanced.html")
            self.logger.info("   • Camera Stream: ws://localhost:8080")
            self.logger.info("   • Police VSS Data: ws://localhost:8081")
            self.logger.info("🚗 Police Units: Active and monitoring")
            self.logger.info("Press Ctrl+C to stop all services...")

            # Keep running until interrupted
            while True:
                time.sleep(10)

                # Check if any process died unexpectedly
                failed_processes = []
                for name, process in self.processes:
                    if process.poll() is not None:
                        failed_processes.append(name)

                if failed_processes:
                    self.logger.warning(f"⚠️ Processes died: {', '.join(failed_processes)}")

                # Could implement auto-restart logic here if needed

        except KeyboardInterrupt:
            self.logger.info("🛑 Shutdown requested by user")
        except Exception as e:
            self.logger.error(f"❌ System error: {e}")
            return False
        finally:
            self.cleanup_processes()

        self.logger.info("✅ Enhanced Police Monitoring System stopped")
        return True

def main():
    launcher = EnhancedPoliceSystemLauncher()
    success = launcher.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()