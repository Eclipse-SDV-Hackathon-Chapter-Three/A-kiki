# GEMINI Analysis for Enhanced Police Monitoring System

## Project Overview

This project is an **Enhanced Police Monitoring System** that integrates several technologies to create a real-time monitoring solution for police vehicles within the CARLA simulator. The system is built upon the Robot Operating System 2 (ROS2), utilizing it as the foundational message-passing layer.

The core of the system's communication is handled by **zenoh**, which serves a dual role:
1.  **VSS Data Transmission:** It routes Vehicle Signal Specification (VSS) compliant police data.
2.  **High-Performance Camera Streaming:** It leverages Shared Memory (SHM) to stream high-resolution camera feeds with low latency.

The entire system is orchestrated by the main script, `run_enhanced_police_system.py`, which manages the lifecycle of all components. A key feature is the web-based dashboard, `police_dashboard_enhanced.html`, which provides a comprehensive user interface for visualizing vehicle status, camera feeds, and emergency alerts.

### Key Technologies

*   **Simulation:** CARLA Simulator
*   **Robotics Middleware:** ROS2 Humble
*   **Communication Protocol:** zenoh (with SHM support)
*   **Data Standard:** Vehicle Signal Specification (VSS)
*   **Frontend:** HTML, CSS, JavaScript (with WebSockets)
*   **Backend/Orchestration:** Python

## Building and Running

### 1. Installation

All dependencies can be installed by running the provided shell script. This script handles system libraries, Python packages, and the compilation of C++ components.

```bash
bash install_dependencies.sh
```

### 2. Running the System

To launch the entire monitoring system, execute the main Python script:

```bash
python3 run_enhanced_police_system.py
```

This command will:
*   Start the zenoh router.
*   Spawn the police vehicle in CARLA.
*   Initiate all data bridges (ROS2 to zenoh, etc.).
*   Launch the WebSocket servers.
*   Automatically open the `police_dashboard_enhanced.html` in your default web browser.

### 3. Individual Components

While the `run_enhanced_police_system.py` script is the recommended way to start the system, individual components can be launched separately for debugging or development purposes. Refer to the "Execution" section in the `README_Technical_Documentation.md` for detailed commands.

## Development Conventions

*   **Modularity:** The system is designed with a modular architecture, where each component has a specific responsibility (e.g., `carla_vehicle_spawner.py`, `kuksa_zenoh_bridge.py`).
*   **Configuration:** System-wide settings are managed through the `config/police_system_config.json` file.
*   **Logging:** The system uses Python's `logging` module to provide detailed, timestamped logs for both console output and file storage.
*   **Process Management:** The main launcher script (`run_enhanced_police_system.py`) is responsible for the graceful startup and shutdown of all processes.
*   **Data Schema:** The VSS data format is defined in `vss_police_schema.json`, ensuring data consistency and validation.
