# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## System Overview

This is the **Enhanced Police Monitoring System** - an integrated CARLA simulation project combining real-time police vehicle monitoring with autonomous chase capabilities. The system integrates CARLA Simulator, ROS2 Humble, Zenoh messaging, and VSS (Vehicle Signal Specification) for a comprehensive vehicle monitoring and pursuit solution.

## Prerequisites

- **Ubuntu 22.04**
- **CARLA Simulator 0.9.15** (expected at `/opt/carla-simulator` or `/home/seame/PythonAPI/`)
- **ROS 2 Humble** - Must source before running: `source /opt/ros/humble/setup.bash`
- **Python 3.8+**
- **Zenoh** messaging framework

## Installation & Setup

```bash
# Install all dependencies (includes Python packages, Rust/zenoh)
bash install_dependencies.sh

# Ensure CARLA is running
cd /opt/carla-simulator && ./CarlaUE4.sh

# Source ROS2 environment (required for every terminal session)
source /opt/ros/humble/setup.bash
```

## Running the System

### Full Police Monitoring System
```bash
# Main launcher - starts entire police monitoring stack
python3 run_enhanced_police_system.py

# Alternative comprehensive launcher
python3 run_police_all.py
```

This launches:
- Zenoh router with shared memory support
- CARLA vehicle spawner (Tesla Model 3 police car)
- VSS data generation and validation
- Camera streaming bridges (WebSocket on port 8080)
- Police data bridge (WebSocket on port 8081)
- Web dashboard (auto-opens in browser)

### Chase System Only
```bash
# Modularized auto-chase controller
cd main/scripts
python3 auto_chase_cmh.py

# Manual vehicle control with chase capabilities
python3 manual_vehicle_control.py
```

### Pedestrian Scenarios
```bash
cd main/scripts
./run_manual_and_pedestrian.sh
```

## Architecture Overview

### Core System Components

**Data Flow Architecture:**
```
CARLA Simulator (port 2000)
    ↓
carla_vehicle_spawner.py (Tesla Model 3 + Sensors)
    ↓
    ├─→ ROS2 Topics (/carla/hero/camera/*, /carla/hero/vehicle_status)
    │   ↓
    │   ros2_camera_zenoh_bridge.py → Zenoh SHM → WebSocket(8080)
    │
    └─→ police/vss_data, police/status
        ↓
        kuksa_zenoh_bridge.py → Zenoh Topics → police_websocket_bridge.py → WebSocket(8081)
            ↓
        police_dashboard_enhanced.html (integrated monitoring)
```

### Module Structure

```
main/
├── chase/                    # Chase system modules
│   ├── perception/          # Object detection, collision detection
│   │   ├── bounding_box_detector.py       # 2D/3D bounding box detection
│   │   ├── collision_detector.py          # Multi-condition collision detection
│   │   ├── collision_vehicle_tracker.py   # Target vehicle tracking
│   │   └── unified_collision_detector.py  # Integrated collision detection
│   │
│   ├── planning/            # Path planning and behavior
│   │   ├── chase_planner.py              # Chase strategy planning
│   │   ├── behavior_planner.py           # High-level behaviors
│   │   └── agents/                       # CARLA agent framework
│   │       └── navigation/               # Global/local planners, controllers
│   │
│   ├── control/             # Vehicle control systems
│   │   ├── auto_chase_controller.py      # Main autonomous chase controller
│   │   ├── vehicle_controller.py         # Low-level vehicle control
│   │   ├── pid_controller.py             # PID control implementation
│   │   └── camera_manager.py             # Multi-camera management
│   │
│   ├── communication/       # Zenoh messaging
│   │   ├── zenoh_manager.py              # Zenoh pub/sub management
│   │   ├── zenoh_camera_publisher.py     # Camera data over Zenoh
│   │   └── zenoh_collision_manager.py    # Collision event distribution
│   │
│   ├── sensors/             # Sensor integration
│   │   ├── lidar_subscriber.py
│   │   └── semantic_lidar_manager.py
│   │
│   └── chase_system.py      # Integrated chase orchestration
│
├── vehicle/                 # Vehicle display and control
│   ├── vehicle_controller.py
│   ├── camera_view.py
│   └── optimized_camera_view.py
│
├── pedestrian/              # Pedestrian spawning and control
│   ├── pedestrian_controller.py
│   └── pedestrian_config.py
│
└── scripts/                 # Execution scripts
    ├── auto_chase_cmh.py                 # Main chase script
    ├── manual_vehicle_control.py         # Manual control with chase toggle
    └── simple_pedestrian_scenario.py     # Pedestrian spawn scenarios
```

### Top-Level System Files

- `run_enhanced_police_system.py` - Main system launcher with dependency checking and graceful shutdown
- `run_police_all.py` - Alternative comprehensive launcher
- `carla_vehicle_spawner.py` - Spawns police vehicle with sensors and publishes to ROS2
- `kuksa_zenoh_bridge.py` - VSS data validation and Zenoh topic bridging
- `police_websocket_bridge.py` - Zenoh → WebSocket bridge for web dashboard
- `ros2_camera_zenoh_bridge.py` - ROS2 camera → Zenoh shared memory bridge
- `police_dashboard_enhanced.html` - Real-time monitoring web interface

## Key Subsystems

### 1. Chase System

**Purpose**: Autonomous vehicle chase with collision detection and target tracking

**Collision Detection Logic** (multi-condition based):
- Aspect ratio change (standing → lying down): >30% change
- Lying down threshold: width > 1.5× height
- Height drop: >0.5m sudden decrease
- Size change: >40% bounding box area change
- **Trigger**: 2+ conditions must be met

**Chase States**:
- `IDLE` - Waiting for collision event
- `SEARCHING` - Scanning for target vehicle
- `APPROACHING` - Moving toward target
- `FOLLOWING` - Active pursuit
- `INTERCEPTING` - Blocking maneuver
- `EMERGENCY` - Emergency stop

**Key Parameters** (in `chase_system.py`):
```python
follow_distance=20.0      # Chase distance (meters)
approach_distance=60.0    # Engagement distance
max_speed=30.0           # Maximum pursuit speed (m/s = 108 km/h)
update_interval=0.05     # Control loop frequency (50ms)
```

### 2. VSS Data System

**Standard**: Vehicle Signal Specification (VSS) compliance
**Schema**: `vss_police_schema.json` (JSON Schema Draft-07)

**VSS Structure**:
```python
{
  "Vehicle": {
    "VehicleIdentification": {...},
    "CurrentLocation": {...},
    "Police": {
      "Status": {...},         # Duty status, availability
      "Emergency": {...},      # Lightbar, siren, code level
      "Equipment": {...},      # Radar, camera, radio status
      "Officer": {...}         # Officer information
    }
  }
}
```

**Emergency Scenarios**:
- `traffic_stop` - CODE1, siren OFF
- `pursuit` - CODE3, siren ON, high speed
- `domestic_call` - CODE2, siren OFF
- `routine_patrol` - All lights OFF

### 3. Zenoh Communication

**Zenoh Topic Structure**:
```
police/{district}/vehicle/{unit_id}/vss          # VSS data stream
police/{district}/vehicle/{unit_id}/status       # Vehicle status
police/{district}/alerts/emergency/{unit_id}     # Emergency alerts
police/{district}/fleet/status                   # Fleet overview
police/{district}/bridge/heartbeat               # System health

carla/police/detection                           # Object detection data
carla/police/collision                           # Collision events
carla/police/chase_command                       # Chase commands
carla/police/vehicle_status                      # Real-time vehicle state
```

**Zenoh Usage Patterns**:
- **Shared Memory (SHM)**: High-bandwidth camera streams (67MB capacity)
- **Regular Messages**: VSS data, commands, status updates
- **Single Router**: Handles both SHM and message traffic simultaneously

### 4. Camera & Display System

**Camera Types** (managed by `camera_manager.py`):
- RGB Camera (front view)
- Semantic Segmentation
- Depth Camera
- LiDAR (optional)

**Display Modes**:
- Single camera view
- Multi-camera split view
- Bounding box overlay
- Target tracking visualization

## Development Commands

### Running Tests
```bash
# Test camera pipeline
python3 test_camera_pipeline.py

# Test unified collision detector
python3 main/chase/perception/test_unified_collision_detector.py
```

### Individual Component Testing
```bash
# CARLA vehicle spawner only
python3 carla_vehicle_spawner.py

# VSS data publisher only
python3 police_vss_data_publisher.py  # DEPRECATED - now integrated in spawner

# KUKSA-Zenoh bridge only
python3 kuksa_zenoh_bridge.py

# WebSocket bridge only
python3 police_websocket_bridge.py --port 8081 --district central
```

### Zenoh Utilities
```bash
# Start Zenoh router manually (if not auto-started)
zenohd --cfg 'transport: {shared_memory: {enabled: true}}'

# Check Zenoh processes
pgrep -f zenohd
ps aux | grep zenoh

# Test Zenoh connectivity
cd zenoh && python3 zenoh_check.py
```

### CARLA Utilities
```bash
# Check CARLA server status
netstat -tlnp | grep :2000
ps aux | grep CarlaUE4

# Spawn additional vehicles
python3 carla_vehicle_spawner.py

# Manual vehicle spawn with custom config
python3 main/scripts/vehicle_spawn_mh3.py
```

## Control Keys (Manual Mode)

**Chase System** (`manual_vehicle_control.py`):
- **ESC** - Exit
- **WASD** - Manual vehicle control
- **C** - Toggle chase mode (manual ↔ autonomous)
- **R** - Reset chase system
- **H** - Toggle HUD display
- **TAB** - Switch camera view

**Chase Auto Mode** (`auto_chase_cmh.py`):
- **ESC** - Exit
- **C** - Manual collision trigger (for testing)
- **S** - Force start chase
- **T** - Stop chase

## Important Implementation Notes

### CARLA-Specific Patterns

**Vehicle Spawning**:
- Always use blueprint library to find vehicle types
- Set autopilot before starting chase logic
- Clean up actors in finally blocks to prevent CARLA memory leaks

**Sensor Attachment**:
- Cameras use relative transforms from vehicle
- Sensors should be destroyed when vehicle is destroyed
- Use weak references for actor storage to prevent circular dependencies

**World Synchronization**:
- Consider using synchronous mode for deterministic testing
- Default is asynchronous mode for real-time performance
- Call `world.tick()` in synchronous mode

### ROS2 Integration

**Topic Naming Convention**:
- `/carla/hero/*` - Main vehicle topics
- `/carla/hero/camera/rgb` - RGB camera feed
- `/carla/hero/vehicle_status` - Vehicle telemetry

**Message Types**:
- Camera: `sensor_msgs/Image`
- Vehicle status: Custom message types
- Zenoh integration uses JSON serialization

### Zenoh Best Practices

**Topic Hierarchy**: Use hierarchical topics for logical grouping (`domain/category/detail`)

**SHM Usage**: Reserved for high-bandwidth data (>10MB) like camera streams

**Message Serialization**: JSON for compatibility, consider MessagePack for performance

**Router Configuration**: Enable shared memory in zenoh config for SHM support

### Performance Considerations

**Update Intervals**:
- Chase control: 50ms (20 Hz)
- Object detection: 100ms (10 Hz)
- VSS publishing: 200ms (5 Hz)
- UI updates: Variable based on WebSocket clients

**Optimization Targets**:
- Minimize camera resolution for detection if FPS drops
- Use frame skipping in bounding box detection
- Batch Zenoh publishes when possible
- Limit tracking history size (default: 30 frames)

## Troubleshooting

### CARLA Connection Issues
```bash
# Verify CARLA is running on correct port
netstat -tlnp | grep :2000

# Check CARLA logs for errors
tail -f /opt/carla-simulator/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log
```

### Zenoh Router Issues
```bash
# Check if zenohd is running
pgrep zenohd

# Restart zenoh router
pkill zenohd
zenohd --cfg 'transport: {shared_memory: {enabled: true}}'
```

### ROS2 Environment Issues
```bash
# Verify ROS2 is sourced
echo $ROS_DISTRO  # Should output "humble"

# Re-source ROS2
source /opt/ros/humble/setup.bash
```

### Module Import Errors
```bash
# Check Python path for CARLA egg
echo $PYTHONPATH | grep carla

# Add CARLA to Python path if missing
export PYTHONPATH="${PYTHONPATH}:/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg"
```

### Performance Issues

**Low FPS**: Reduce camera resolution, disable unnecessary sensors, use simpler vehicle models

**Zenoh Latency**: Check network settings, verify SHM is enabled for large messages

**Chase System Lag**: Increase update_interval, reduce detection frequency, simplify tracking logic

## System Ports

- **2000** - CARLA Simulator server
- **8080** - Camera stream WebSocket
- **8081** - Police VSS data WebSocket
- **7447** - Zenoh router (default)

## File Naming Conventions

- `*_controller.py` - Control systems (PID, vehicle, chase)
- `*_detector.py` - Perception modules (collision, objects)
- `*_manager.py` - System management (camera, zenoh, sensors)
- `*_planner.py` - Planning algorithms (path, behavior, chase)
- `*_publisher.py` - Data publishers (Zenoh, ROS2)
- `*_subscriber.py` - Data subscribers
- `auto_*.py` - Autonomous operation scripts
- `manual_*.py` - Manual control scripts

## Configuration Files

- `config/police_system_config.json` - Main system configuration (optional)
- `vss_police_schema.json` - VSS validation schema
- `package.xml` - ROS2 package manifest

## Logging

**Log Files**: `enhanced_police_system_{timestamp}.log`

**Log Levels**: DEBUG, INFO, WARNING, ERROR

**Performance Metrics Tracked**:
- Messages received/published
- Validation errors
- Emergency alerts triggered
- Active units count
- System uptime

## Web Dashboard

**URL**: `file://police_dashboard_enhanced.html` (auto-opens on system start)

**Features**:
- Real-time camera feed (WebSocket 8080)
- VSS data visualization (WebSocket 8081)
- Vehicle location tracking
- Emergency alerts
- Equipment status monitoring

## Extension Points

### Adding New Sensors
1. Define sensor in `carla_vehicle_spawner.py`
2. Extend VSS schema in `vss_police_schema.json`
3. Update data publisher logic
4. Add Zenoh topic for sensor data

### Adding Chase Behaviors
1. Extend `ChasePlanner` in `chase/planning/chase_planner.py`
2. Add new state to chase state machine
3. Implement behavior logic in controller
4. Update UI to display new behavior

### Multi-Vehicle Support
1. Modify `carla_vehicle_spawner.py` for multiple units
2. Use district-based Zenoh topics: `police/{district}/vehicle/{unit_id}/...`
3. Update web dashboard for multi-vehicle display
4. Implement vehicle coordination logic

## Dependencies Reference

**Python Packages**:
- `carla` - CARLA Python API
- `eclipse-zenoh` - Zenoh messaging
- `rclpy` - ROS2 Python client
- `jsonschema` - VSS validation
- `websockets` - WebSocket server
- `opencv-python` - Image processing
- `numpy` - Numerical operations
- `pygame` - Input handling and display

**System Dependencies**:
- ROS2 Humble (`ros-humble-*`)
- Zenoh router (`zenohd`)
- Rust toolchain (for Zenoh build)

## Security Notes

- WebSocket connections are currently unencrypted (WS not WSS)
- Zenoh router authentication is not configured by default
- For production: Enable TLS/WSS and configure Zenoh access control
- VSS data validation prevents malformed inputs
