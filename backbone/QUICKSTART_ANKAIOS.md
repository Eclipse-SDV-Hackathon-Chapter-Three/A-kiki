# Quick Start Guide - Ankaios Deployment

Get the Enhanced Police Monitoring System running with Ankaios in 5 minutes.

## Prerequisites Check

```bash
# Check if Podman is installed
podman --version
# If not: sudo apt install podman

# Check if CARLA is installed
ls /opt/carla-simulator/  # or your CARLA installation path
```

## Step 1: Install Ankaios (One-time)

```bash
cd /home/seame/Workspace/ros-bridge/Carla_ROS2_KUKSA_ZENOH_WEB

# Run the installer
./install_ankaios.sh
```

**Expected output:**
```
📦 Installing Eclipse Ankaios v0.5.0...
⬇️  Downloading Ankaios binaries...
✅ Downloaded successfully
📥 Installing binaries to /usr/local/bin...
✅ Binaries installed
🎉 Eclipse Ankaios v0.5.0 installed successfully!
```

## Step 2: Build Container Images (One-time, ~10 minutes)

```bash
# Build all images
./build_containers.sh
```

**Expected output:**
```
🐳 Building Police Monitoring System Containers...
📦 Building base image...
✅ Base image built successfully
🚗 Building CARLA spawner image...
✅ CARLA spawner image built successfully
...
🎉 All container images built successfully!
```

Verify images:
```bash
podman images | grep police
```

You should see:
- `localhost/police-base`
- `localhost/police-carla-spawner`
- `localhost/police-kuksa-bridge`
- `localhost/police-camera-bridge`
- `localhost/police-websocket`

## Step 3: Start CARLA Simulator

```bash
# In a new terminal
cd /opt/carla-simulator/  # or your CARLA path
./CarlaUE4.sh
```

Wait for CARLA to fully load (you'll see the town/city rendered).

## Step 4: Start the System

```bash
# Back in your original terminal
./start_ankaios.sh
```

**Expected output:**
```
🚔 Starting Enhanced Police Monitoring System with Ankaios...
🔍 Checking CARLA simulator...
✅ All container images found
🚀 Starting Ankaios server...
✅ Ankaios server started
🤖 Starting Ankaios agent...
✅ Ankaios agent started
📋 Applying workload manifest...
✅ Workload manifest applied successfully
🎉 Enhanced Police Monitoring System started successfully!
```

## Step 5: Access the Dashboard

```bash
# Open in browser
firefox police_dashboard_enhanced.html
```

Or click the file:// URL shown in the terminal output.

## Verify System is Running

```bash
# Check workload status
ank -k get workloads

# Check running containers
podman ps
```

You should see 6 containers running:
1. `zenoh-router`
2. `carla-vss-spawner`
3. `kuksa-zenoh-bridge`
4. `camera-zenoh-bridge`
5. `camera-websocket`
6. `police-websocket`

## View Logs

```bash
# Follow logs from a component
podman logs -f carla-vss-spawner

# View all logs
podman logs zenoh-router
podman logs kuksa-zenoh-bridge
podman logs camera-websocket
```

## Stop the System

```bash
./stop_ankaios.sh
```

## Troubleshooting

### "CARLA not detected"
Make sure CARLA is running: `nc -z localhost 2000`

### "Missing container images"
Run: `./build_containers.sh`

### "Ankaios CLI not found"
Run: `./install_ankaios.sh`

### Workload fails to start
Check logs: `podman logs <container-name>`

## What's Running?

```
Component              Port    Purpose
────────────────────────────────────────────────────
zenoh-router           -       Communication middleware
carla-vss-spawner      -       Spawns police vehicle
kuksa-zenoh-bridge     -       VSS data processing
camera-zenoh-bridge    -       Camera stream processing
camera-websocket       8080    Camera stream to web
police-websocket       8081    Police data to web
Dashboard              -       Web UI (file://)
```

## Next Steps

- Check `ANKAIOS_README.md` for detailed configuration
- Customize `ankaios_manifest.yaml` for your needs
- Add new workloads following the examples

## Common Commands

```bash
# List workloads
ank -k get workloads

# View workload details
ank -k get workload zenoh-router

# Restart a component
ank -k delete workload camera-websocket
# Ankaios will restart it automatically

# View system state
ank -k get state
```

## Support

For issues, check:
1. `ANKAIOS_README.md` - Detailed documentation
2. `/tmp/ankaios-server.log` - Server logs
3. `/tmp/ankaios-agent.log` - Agent logs
4. Container logs with `podman logs <name>`
