# Installation Guide for Enhanced Police Monitoring System

This guide explains how to set up and run the Enhanced Police Monitoring System.

## 1. Prerequisites

Before you begin, please ensure you have the following installed on your **Ubuntu 22.04** system:

*   **CARLA Simulator:** The system is designed to connect to a running CARLA instance. Please download and install it from the official website.
*   **ROS 2 Humble:** This project depends on ROS 2 Humble. Please follow the official ROS 2 documentation to install it. Make sure to source the setup file (e.g., `source /opt/ros/humble/setup.bash`).

## 2. Installation

Once the prerequisites are met, you can install all the required dependencies and build the project components by running the provided installation script.

Open your terminal in the project root directory and execute the following command:

```bash
bash install_dependencies.sh
```

This script will automatically:
1.  Install necessary system libraries (like Boost and nlohmann-json).
2.  Install required Python packages using pip.
3.  Compile the C++ components of the project.

## 3. Running the System

After the installation script completes successfully, you can launch the entire system with a single command:

```bash
python3 run_enhanced_police_system.py
```

This will start all the necessary modules, including the bridges and data publishers, and should automatically open the web dashboard in your browser.
