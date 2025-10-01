# Police Scripts

This directory contains executable scripts for CARLA police scenarios.

## Available Scripts

### Simple Pedestrian Scenario

A basic pedestrian walking scenario without vehicles.

**Files:**
- `simple_pedestrian_scenario.py` - Main scenario script
- `run_pedestrian_scenario.sh` - Execution script

**Usage:**
```bash
# From police/main/scripts directory
./run_pedestrian_scenario.sh

# Or directly with Python
python3 simple_pedestrian_scenario.py
```

**Features:**
- Pedestrian walks automatically between start and end points
- Configurable walking speed (currently set to 15 m/s)
- No vehicle interactions
- Clean, modular code structure

### Manual Vehicle Control

Independent manual vehicle control for separate terminal execution.

**Files:**
- `manual_vehicle_control.py` - Main manual vehicle control script
- `run_manual_vehicle.sh` - Execution script

**Usage:**
```bash
# From police/main/scripts directory
./run_manual_vehicle.sh

# Or directly with Python
python3 manual_vehicle_control.py
```

**Features:**
- Manual vehicle control with keyboard (WASD)
- Real-time camera view with OpenCV
- HUD display showing position, speed, and controls
- Camera positioned for optimal driving view
- Real-time status display in terminal
- Independent execution (can run alongside pedestrian scenario)
- Clean, modular code structure

### Chase Vehicle Control

Independent chase vehicle control for separate terminal execution.

**Files:**
- `chase_vehicle_control.py` - Main chase vehicle control script
- `run_chase_vehicle.sh` - Execution script

**Usage:**
```bash
# From police/main/scripts directory
./run_chase_vehicle.sh

# Or directly with Python
python3 chase_vehicle_control.py
```

**Features:**
- Autonomous chase vehicle control
- Multi-sensor fusion (Camera, LiDAR, Radar)
- Intelligent path planning and control
- Real-time target tracking
- Modular perception, planning, and control modules
- Independent execution (can run alongside manual vehicle)
- Clean, modular code structure

**Requirements:**
- CARLA 0.9.15 running on localhost:2000
- Python 3.8+
- ROS 2 (for node functionality)

**Controls:**
- Ctrl+C to quit

**Directory Structure:**
```
police/
├── main/
│   ├── pedestrian/          # Pedestrian module
│   │   ├── pedestrian_controller.py
│   │   ├── pedestrian_config.py
│   │   └── __init__.py
│   └── scripts/             # Executable scripts
│       ├── simple_pedestrian_scenario.py
│       ├── run_pedestrian_scenario.sh
│       └── __init__.py
```
