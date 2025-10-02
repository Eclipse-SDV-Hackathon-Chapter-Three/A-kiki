# Backbone System - Enhanced Police Monitoring Infrastructure

## Overview

The Backbone system is the core infrastructure for police vehicle monitoring, integrating CARLA Simulator, ROS2, and Zenoh messaging to handle real-time vehicle data collection, processing, and transmission.

## System Architecture

```
CARLA Simulator (Port 2000)
    ↓
carla_vehicle_spawner.py (Police Vehicle + Sensors)
    ↓
    ├─→ ROS2 Topics (/carla/hero/*)
    │   ↓
    │   ros2_camera_zenoh_bridge.py → Zenoh → WebSocket(8080)
    │
    └─→ VSS Data (police/vss_data)
        ↓
        kuksa_zenoh_bridge.py → Zenoh Topics → police_websocket_bridge.py → WebSocket(8081)
            ↓
        police_dashboard_enhanced.html (Web Dashboard)
```

## Core Components

### 1. Vehicle Spawner & Sensor Management
**File**: `carla_vehicle_spawner.py`

#### Features
- Spawns Dodge Charger police vehicle in CARLA
- Manages multiple sensors:
  - RGB Camera (front view)
  - Semantic LiDAR (object detection)
  - GNSS (GPS positioning)
  - IMU (orientation/acceleration)
  - Map Camera (bird's eye view)

#### Data Generation
- **ROS2 Topics**:
  - `/carla/hero/camera/image/compressed` - Compressed front camera feed
  - `/carla/hero/semantic_lidar/point_cloud` - Point cloud data
  - `/carla/hero/gnss` - GPS coordinates
  - `/carla/hero/vehicle_status` - Vehicle telemetry
  - `/carla/map_camera/image/compressed` - Map camera feed

- **Zenoh Topics**:
  - `target_gps` - Detected objects (pedestrians/vehicles) with GPS and NED coordinates
  - `lidar_gps` - Police vehicle's own position
  - `imu` - IMU sensor data

#### Object Detection Logic
- Object classification via Semantic LiDAR (Tag 12: Pedestrian, 14: Vehicle)
- ObjIdx-based clustering for individual object tracking
- Minimum point filtering (default: 10+ points)
- Ego exclusion zone around vehicle

#### VSS Data Generation
- Vehicle Signal Specification (VSS) standard compliance
- Police-specific extended fields:
  - Emergency equipment status (lightbar, siren)
  - Officer information (badge number, name)
  - Active call details
  - Equipment status (dash cam, body cam, MDT)

#### Environment Variables
```bash
CARLA_HOST=localhost               # CARLA server address
CARLA_PORT=2000                    # CARLA port
PUB_INTERVAL_SEC=0.3              # Data publishing interval (seconds)
SEM_LIDAR_STRIDE=1                # LiDAR downsampling stride
INCLUDE_TAGS="12,14"              # Object tags to include (12=Pedestrian, 14=Car)
MIN_CLUSTER_POINTS=10             # Minimum points per cluster
REMOVE_REAR_POINTS=true           # Remove rear-facing points
EGO_EXCLUDE_RADIUS=2.5            # Ego exclusion radius (meters)
```

---

### 2. VSS Data Validator & ROS2-Zenoh Bridge
**File**: `kuksa_zenoh_bridge.py`

> **Note**: Despite the filename, this component does **NOT** use KUKSA. It's a pure ROS2-to-Zenoh bridge with VSS validation using standard JSON Schema.

#### Features
- Forwards VSS data from ROS2 to Zenoh network
- VSS schema-based data validation (using `jsonschema`, not KUKSA)
- Emergency situation detection and alert generation

#### Zenoh Topic Structure
```
police/{district}/vehicle/{unit_id}/vss          # VSS data stream
police/{district}/vehicle/{unit_id}/status       # Vehicle status
police/{district}/alerts/emergency/{unit_id}     # Emergency alerts
police/{district}/fleet/status                   # Fleet-wide status
police/{district}/bridge/heartbeat               # System health check
```

#### Emergency Detection
- **Panic Button Activation** (CRITICAL priority)
- **Emergency Equipment Activation** (lightbar/siren turned on)
- **High Priority Call Assignment** (EMERGENCY/CRITICAL status)

#### Data Validation
- JSON Schema validation based on `vss_police_schema.json`
- Invalid data is logged and discarded

#### Parameters
```bash
--ros-args \
  -p police_district:=central \
  -p enable_data_validation:=true \
  -p emergency_alert_topics:=true
```

#### Performance Monitoring
- Performance statistics logged every 30 seconds:
  - Messages received/published
  - Validation errors
  - Emergency alerts
  - Active units count

---

### 3. Camera Video Streaming
**Files**: `ros2_camera_zenoh_bridge.py`, `camera_websocket_server.py`

#### ros2_camera_zenoh_bridge.py
- Forwards ROS2 CompressedImage → Zenoh
- Uses JPEG-compressed images from CARLA directly (no re-encoding)
- Handles two camera streams:
  - Front camera: `camera/front_rgb/compressed`
  - Map camera: `camera/map/compressed`

#### camera_websocket_server.py
- Broadcasts Zenoh camera feeds via WebSocket (port 8080)
- Wildcard subscription: `camera/**/compressed`
- Message format:
  ```
  [4 bytes: header length] + [JSON header] + [JPEG binary data]

  JSON header example:
  {
    "camera": "front_rgb",
    "timestamp": "2025-10-02T12:34:56.789",
    "size": 45678
  }
  ```

#### Performance Statistics
- Frame rate and bandwidth reported every 5 seconds
- Per-client connection tracking

---

### 4. VSS Data WebSocket Bridge
**File**: `police_websocket_bridge.py`

#### Features
- Receives police VSS data from Zenoh
- Real-time transmission to web dashboard via WebSocket (port 8081)
- Supports multiple concurrent clients

#### Message Types
- `vss_data` - Vehicle VSS data
- `status` - Vehicle status updates
- `emergency_alert` - Emergency notifications
- `fleet_status` - Fleet-wide status
- `heartbeat` - Bridge health check
- `suspect_gps` - Suspect location (chase system integration)

#### New Client Handling
- Welcome message on connection
- Sends cached latest data (initial sync)

#### Client Commands
- `ping` → `pong` response
- `status_request` → Server status response

#### Parameters
```bash
--port 8081               # WebSocket port
--district central        # Police district
```

---

### 5. Integrated System Launcher
**File**: `run_enhanced_police_system.py`

#### Features
- Sequentially starts all backbone components
- Dependency verification (CARLA, ROS2, Zenoh, Python packages)
- Configuration file loading (`config/police_system_config.json`)
- Graceful shutdown handling

#### Startup Sequence
1. **Zenoh Router** - With SHM support enabled
2. **CARLA Vehicle Spawner** - Police vehicle + sensors + VSS
3. **VSS-Zenoh Bridge** - VSS data validation and forwarding (no KUKSA involved)
4. **Camera Zenoh Bridge** - Video data forwarding
5. **WebSocket Bridges** - Camera (8080) + VSS (8081)
6. **Web Dashboard** - Auto-open in browser

#### Configuration File Example
```json
{
  "camera_shm_capacity": 67108864,
  "police_district": "central",
  "enable_vss_validation": true,
  "enable_emergency_alerts": true,
  "carla_host": "localhost",
  "carla_port": 2000,
  "police_units": [
    {
      "unit_id": "UNIT-001",
      "officer_primary": {
        "name": "Officer Johnson",
        "badge": "12345"
      }
    }
  ],
  "dashboard_auto_open": true,
  "log_level": "INFO"
}
```

#### Process Monitoring
- Status check for each process every 10 seconds
- Detects and logs unexpected terminations

#### Shutdown Handling
- SIGINT/SIGTERM signal handling
- Clean up all processes (SIGTERM → SIGKILL)
- Zenoh session cleanup

---

## VSS Schema Structure

**File**: `vss_police_schema.json`

### Key Fields
```json
{
  "Vehicle": {
    "VehicleIdentification": {
      "VIN": "Vehicle Identification Number",
      "Model": "Model name",
      "Police": {
        "UnitNumber": "Unit number (UNIT-001)",
        "District": "Jurisdiction",
        "Precinct": "Police precinct"
      }
    },
    "CurrentLocation": {
      "Latitude": "Latitude",
      "Longitude": "Longitude",
      "Altitude": "Altitude",
      "Heading": "Heading angle"
    },
    "Police": {
      "Status": {
        "Availability": "AVAILABLE|BUSY|EMERGENCY"
      },
      "Emergency": {
        "Lightbar": {"IsActive": true/false},
        "Siren": {"IsActive": true/false},
        "PanicButton": {"IsPressed": true/false}
      },
      "Equipment": {
        "DashCamera": {"IsRecording": true/false},
        "MDT": {"ActiveCall": {...}},
        "BodyCamera": {
          "Officer1": {"IsActive": true, "BatteryLevel": 85}
        }
      },
      "Officer": {
        "Primary": {
          "BadgeNumber": "12345",
          "Name": "Officer Johnson",
          "Status": "ON_DUTY"
        }
      }
    }
  }
}
```

---

## Usage

### Run Complete System
```bash
# Install dependencies
bash install_dependencies.sh

# Activate ROS2 environment
source /opt/ros/humble/setup.bash

# Start entire system
python3 run_enhanced_police_system.py
```

### Run Individual Components
```bash
# Zenoh router (manual)
zenohd --cfg 'transport: {shared_memory: {enabled: true}}'

# CARLA vehicle spawner
python3 carla_vehicle_spawner.py

# VSS-Zenoh bridge (not actually using KUKSA, just named that way)
python3 kuksa_zenoh_bridge.py --ros-args \
  -p police_district:=central \
  -p enable_data_validation:=true

# Camera bridge
python3 ros2_camera_zenoh_bridge.py --zenoh-mode peer

# Camera WebSocket server
python3 camera_websocket_server.py --port 8080

# VSS WebSocket bridge
python3 police_websocket_bridge.py --port 8081 --district central
```

---

## Network Ports

| Port | Service | Protocol |
|------|---------|----------|
| 2000 | CARLA Simulator | TCP |
| 7447 | Zenoh Router | TCP/SHM |
| 8080 | Camera Stream | WebSocket |
| 8081 | VSS Data | WebSocket |

---

## Zenoh Topic Structure

### Vehicle Data
```
police/central/vehicle/UNIT-001/vss          # Full VSS data
police/central/vehicle/UNIT-001/status       # Status updates
```

### Alerts
```
police/central/alerts/emergency/UNIT-001     # Emergency alerts
```

### System
```
police/central/fleet/status                  # Fleet status
police/central/bridge/heartbeat              # Health check
```

### Sensor Data
```
target_gps                                   # Detected objects GPS
lidar_gps                                    # Vehicle GPS
imu                                          # IMU data
camera/front_rgb/compressed                  # Front camera
camera/map/compressed                        # Map camera
```

---

## Troubleshooting

### CARLA Connection Failed
```bash
# Check if CARLA is running
netstat -tlnp | grep :2000

# Restart CARLA
cd /opt/carla-simulator && ./CarlaUE4.sh
```

### Zenoh Router Issues
```bash
# Check Zenoh process
pgrep zenohd

# Restart Zenoh
pkill zenohd
zenohd --cfg 'transport: {shared_memory: {enabled: true}}'
```

### ROS2 Environment Error
```bash
# Verify ROS2 is sourced
echo $ROS_DISTRO  # Should output "humble"

# Re-source
source /opt/ros/humble/setup.bash
```

### WebSocket Connection Failed
```bash
# Check port usage
netstat -tlnp | grep 8080
netstat -tlnp | grep 8081

# Check firewall
sudo ufw status
```

---

## Performance Optimization

### Camera Streaming
- Adjust resolution: Change `image_size_x`, `image_size_y` in `carla_vehicle_spawner.py`
- JPEG quality: Adjust `encode_param` value (currently 90%)

### LiDAR Processing
- Downsampling: Increase `SEM_LIDAR_STRIDE` environment variable
- Filtering: Increase `MIN_CLUSTER_POINTS` to reduce noise

### Zenoh Bandwidth
- Verify SHM usage (for large data)
- Adjust publishing interval: `PUB_INTERVAL_SEC` environment variable

---

## Log Files

- **System Launcher**: `enhanced_police_system_YYYYMMDD_HHMMSS.log`
- **Individual Components**: Output to stdout/stderr

---

## Development Tips

### Modify VSS Schema
1. Edit `vss_police_schema.json`
2. Restart `kuksa_zenoh_bridge.py`
3. Check validation errors in bridge logs

### Add New Sensor
1. Define sensor in `carla_vehicle_spawner.py`
2. Write callback function
3. Add ROS2/Zenoh publisher

### Add Emergency Scenario
1. Edit `_initialize_emergency_scenarios()` in `carla_vehicle_spawner.py`
2. Add new scenario definition
3. Extend `_apply_scenario()` logic

---

## Dependencies

### Python Packages
- `carla` - CARLA Python API
- `eclipse-zenoh` - Zenoh messaging
- `rclpy` - ROS2 Python client
- `jsonschema` - VSS validation
- `websockets` - WebSocket server
- `opencv-python` - Image processing
- `numpy` - Numerical operations

### System Dependencies
- ROS2 Humble (`ros-humble-*`)
- Zenoh router (`zenohd`)
- CARLA Simulator 0.9.15

---

## License

This system is part of the Eclipse SDV Hackathon Chapter Three.
