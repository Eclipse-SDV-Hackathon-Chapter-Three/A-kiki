# Enhanced Police Monitoring System - Ankaios Deployment

This directory contains the Ankaios-based orchestration configuration for the Enhanced Police Monitoring System, providing containerized workload management for automotive HPC platforms.

## 🏗️ Architecture

The system uses **Eclipse Ankaios** for workload orchestration with the following components:

```
┌─────────────────────────────────────────────────────────────┐
│                    Ankaios Server                           │
│                 (Orchestration Control)                     │
└─────────────────────┬───────────────────────────────────────┘
                      │
         ┌────────────┴────────────┐
         │   Ankaios Agent         │
         │   (police_agent)        │
         └─────────┬───────────────┘
                   │
       ┌───────────┼──────────────────────────┐
       │           │                          │
   Podman      Podman                    Podman
  Runtime     Runtime                   Runtime
       │           │                          │
  ┌────┴────┬─────┴─────┬──────────────────┬─┴──────┐
  │ zenoh   │ CARLA     │ KUKSA-zenoh      │ Camera │ WebSocket
  │ router  │ spawner   │ bridge           │ bridge │ bridges
  └─────────┴───────────┴──────────────────┴────────┴─────────┘
```

### Workload Dependencies

```
zenoh-router (ALWAYS running)
    ↓
    ├→ carla-vss-spawner (ON_FAILURE restart)
    │      ↓
    │      ├→ kuksa-zenoh-bridge → police-websocket
    │      └→ camera-zenoh-bridge → camera-websocket
```

## 📋 Prerequisites

### 1. Ankaios Installation

Install Eclipse Ankaios following the official guide:

```bash
# Download and install Ankaios (adjust version as needed)
wget https://github.com/eclipse-ankaios/ankaios/releases/download/v0.5.0/ank-linux-x86_64
wget https://github.com/eclipse-ankaios/ankaios/releases/download/v0.5.0/ank-server-linux-x86_64
wget https://github.com/eclipse-ankaios/ankaios/releases/download/v0.5.0/ank-agent-linux-x86_64

chmod +x ank-*
sudo mv ank-linux-x86_64 /usr/local/bin/ank
sudo mv ank-server-linux-x86_64 /usr/local/bin/ank-server
sudo mv ank-agent-linux-x86_64 /usr/local/bin/ank-agent
```

Verify installation:
```bash
ank --version
ank-server --version
ank-agent --version
```

### 2. Podman Installation

```bash
# Ubuntu 22.04
sudo apt-get update
sudo apt-get install -y podman

# Verify
podman --version
```

### 3. CARLA Simulator

Ensure CARLA v0.9.15 is installed and running:
```bash
# CARLA should be listening on localhost:2000
cd /path/to/CARLA_0.9.15
./CarlaUE4.sh
```

### 4. System Dependencies

All ROS2, Python, and zenoh dependencies are included in the container images.

## 🚀 Quick Start

### Step 1: Build Container Images

```bash
cd /home/seame/Workspace/ros-bridge/Carla_ROS2_KUKSA_ZENOH_WEB

# Make build script executable
chmod +x build_containers.sh

# Build all images (takes 5-10 minutes first time)
./build_containers.sh
```

This creates the following images:
- `localhost/police-base:latest` - Base image with ROS2 + Python + zenoh
- `localhost/police-carla-spawner:latest` - CARLA vehicle spawner
- `localhost/police-kuksa-bridge:latest` - KUKSA-zenoh bridge
- `localhost/police-camera-bridge:latest` - Camera zenoh bridge
- `localhost/police-websocket:latest` - WebSocket servers

### Step 2: Start CARLA Simulator

```bash
# In a separate terminal
cd /path/to/CARLA_0.9.15
./CarlaUE4.sh
```

### Step 3: Start Ankaios System

```bash
# Make start script executable
chmod +x start_ankaios.sh

# Start the system
./start_ankaios.sh
```

This will:
1. Check prerequisites (CARLA, containers, Ankaios)
2. Start Ankaios server and agent
3. Apply the workload manifest
4. Start all components in the correct order

### Step 4: Access the System

Open the dashboard in your browser:
```bash
# The script will show the file:// URL
# Or open manually:
firefox police_dashboard_enhanced.html
```

**Access Points:**
- 📹 Camera Stream: `ws://localhost:8080`
- 🚓 Police VSS Data: `ws://localhost:8081`
- 🖥️ Dashboard: `file://<path>/police_dashboard_enhanced.html`

## 🛠️ Management Commands

### View System Status

```bash
# List all workloads
ank -k get workloads

# Get detailed workload info
ank -k get workload <workload-name>

# View workload states
ank -k get state
```

### View Logs

```bash
# View logs for a specific component
podman logs zenoh-router
podman logs carla-vss-spawner
podman logs kuksa-zenoh-bridge
podman logs camera-zenoh-bridge
podman logs camera-websocket
podman logs police-websocket

# Follow logs in real-time
podman logs -f <container-name>
```

### Restart a Workload

```bash
# Delete the workload (Ankaios will restart based on policy)
ank -k delete workload <workload-name>

# Manually restart container
podman restart <container-name>
```

### Stop the System

```bash
# Make stop script executable
chmod +x stop_ankaios.sh

# Stop all workloads
./stop_ankaios.sh
```

## 📄 Configuration Files

### `ankaios_manifest.yaml`

Main workload configuration file defining all system components:
- Workload definitions
- Container runtime configuration
- Dependency relationships
- Restart policies
- Resource requirements

**Key Fields:**
- `apiVersion`: Ankaios API version (v0.1)
- `workloads`: Dictionary of workload definitions
- `runtime`: Container runtime (podman)
- `agent`: Target agent name
- `restartPolicy`: ALWAYS, ON_FAILURE, or NEVER
- `dependencies`: Workload startup dependencies
- `runtimeConfig`: Podman-specific configuration

### Dockerfiles

- `Dockerfile.base` - Base image with all dependencies
- `Dockerfile.carla-spawner` - CARLA vehicle spawner
- `Dockerfile.kuksa-bridge` - KUKSA-zenoh bridge
- `Dockerfile.camera-bridge` - Camera zenoh bridge
- `Dockerfile.websocket` - WebSocket servers

### Scripts

- `build_containers.sh` - Build all container images
- `start_ankaios.sh` - Start the Ankaios-managed system
- `stop_ankaios.sh` - Stop the system and clean up

## 🔧 Customization

### Modify Workload Configuration

Edit `ankaios_manifest.yaml`:

```yaml
workloads:
  my-workload:
    runtime: podman
    agent: police_agent
    restartPolicy: ON_FAILURE
    runtimeConfig: |
      image: localhost/my-image:latest
      commandOptions: ["--network", "host"]
      commandArgs: ["python3", "/app/script.py"]
```

Apply changes:
```bash
ank -k apply ankaios_manifest.yaml
```

### Change Police District

Edit the `police-websocket` workload in `ankaios_manifest.yaml`:

```yaml
commandArgs: [
  "python3", "/workspace/police_websocket_bridge.py",
  "--port", "8081",
  "--district", "north"  # Change from "central" to "north"
]
```

### Add New Workload

1. Create Dockerfile for the component
2. Build the image
3. Add workload definition to `ankaios_manifest.yaml`
4. Apply the updated manifest

## 🐛 Troubleshooting

### Workload Not Starting

```bash
# Check workload state
ank -k get workload <workload-name>

# Check container logs
podman logs <container-name>

# Check if image exists
podman images | grep police
```

### Connection Issues

```bash
# Verify CARLA is running
nc -z localhost 2000

# Check network connectivity
podman network ls
podman network inspect podman

# Verify zenoh router is running
podman ps | grep zenoh-router
```

### Container Build Failures

```bash
# Clean and rebuild
podman system prune -a
./build_containers.sh
```

### Ankaios Server/Agent Issues

```bash
# Check if running
pgrep -a ank-server
pgrep -a ank-agent

# View logs
tail -f /tmp/ankaios-server.log
tail -f /tmp/ankaios-agent.log

# Restart components
pkill ank-server ank-agent
./start_ankaios.sh
```

## 📊 Performance Monitoring

### Resource Usage

```bash
# Monitor container resources
podman stats

# Check system resources
htop

# View podman system info
podman system df
```

### Network Performance

```bash
# Monitor network traffic
iftop

# Check port bindings
ss -tulpn | grep -E '(8080|8081|2000)'
```

## 🔒 Security Considerations

- Containers run with host network for performance
- Volume mounts are labeled with `:Z` for SELinux compatibility
- No privileged containers by default
- Use `ControlInterfaceAccess` in manifest for workload API access control

## 📚 Additional Resources

- [Ankaios Documentation](https://eclipse-ankaios.github.io/ankaios/latest/)
- [Ankaios GitHub](https://github.com/eclipse-ankaios/ankaios)
- [Podman Documentation](https://docs.podman.io/)
- [CARLA Documentation](https://carla.readthedocs.io/)

## 🆚 Comparison: Ankaios vs Direct Launch

### Traditional Launch (`run_enhanced_police_system.py`)

**Pros:**
- Simple setup
- Direct process management
- Easy debugging

**Cons:**
- Manual dependency management
- No containerization
- Limited fault tolerance
- Hard to scale across nodes

### Ankaios-based Launch

**Pros:**
- Container isolation and reproducibility
- Automatic restart policies
- Dependency-aware startup
- Multi-node support
- Better resource management
- Automotive-grade orchestration

**Cons:**
- More complex setup
- Container overhead
- Requires Ankaios installation

## 💡 Tips

1. **First Time Setup**: Allow extra time for container image downloads and builds
2. **Development**: Use volume mounts to update Python code without rebuilding
3. **Production**: Create tagged image versions instead of `:latest`
4. **Logging**: Configure podman to use journald for better log management
5. **Performance**: Use host network mode for low-latency communication

## 🤝 Contributing

When adding new components:
1. Create appropriate Dockerfile
2. Add to `build_containers.sh`
3. Define workload in `ankaios_manifest.yaml`
4. Update this README

## 📝 License

See main project LICENSE file.
