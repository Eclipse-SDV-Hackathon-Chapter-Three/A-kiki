# Gemini Code Understanding

## Project Overview

This project is an "Enhanced Police Monitoring System" that uses the CARLA simulator to provide real-time police vehicle monitoring. It's built primarily in Python and integrates several technologies, including ROS2 for inter-process communication, Zenoh for high-performance data transfer (including shared memory for camera streams), and a web-based dashboard for visualization.

The system is designed to be modular, with different components responsible for specific tasks:

*   **CARLA Vehicle Spawner:** Spawns police vehicles in the CARLA simulator and publishes their data (including camera feeds and vehicle status) to ROS2 topics.
*   **KUKSA-Zenoh Bridge:** Bridges the gap between the KUKSA vehicle data standard and the Zenoh communication protocol. It validates VSS (Vehicle Signal Specification) data and handles emergency alerts.
*   **WebSocket Bridges:** Expose the Zenoh data streams (camera and vehicle data) to the web dashboard via websockets.
*   **Web Dashboard:** A web-based interface (`police_dashboard_enhanced.html`) that displays the real-time location, status, and camera feed of the police vehicles.
*   **C++ Components:** While the core logic is in Python, there are C++ components for high-performance tasks, such as the shared memory bridge for camera data.

## Building and Running

### 1. Installation

First, ensure you have the prerequisites installed, as detailed in `README_INSTALL.md` (CARLA Simulator and ROS 2 Humble). Then, run the installation script to install all the necessary dependencies:

```bash
bash install_dependencies.sh
```

This script will install system packages, Python libraries, and compile the C++ components.

### 2. Running the System

To run the entire system, execute the main launcher script:

```bash
python3 run_enhanced_police_system.py
```

This script orchestrates the startup of all the necessary components in the correct order:

1.  Zenoh router (with shared memory support)
2.  CARLA vehicle spawner
3.  KUKSA-Zenoh bridge
4.  Camera-Zenoh bridge
5.  WebSocket bridges

It will also automatically open the web dashboard in your browser.

## Development Conventions

*   **Configuration:** The system is configured via the `config/police_system_config.json` file. This file allows you to set parameters such as the CARLA host and port, police unit details, and enable/disable features like VSS validation and emergency alerts.
*   **Logging:** The system uses Python's `logging` module to provide detailed logs. Logs are printed to the console and also saved to a file (e.g., `enhanced_police_system_YYYYMMDD_HHMMSS.log`).
*   **Modularity:** The project is structured in a modular way, with different components in separate files and directories. This makes it easier to understand, maintain, and extend the system.
*   **VSS Standard:** The system adheres to the Vehicle Signal Specification (VSS) standard for vehicle data. The VSS schema is defined in `vss_police_schema.json`.
