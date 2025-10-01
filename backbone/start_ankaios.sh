#!/bin/bash
# Start Ankaios-managed Police Monitoring System

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST_FILE="${SCRIPT_DIR}/ankaios_manifest.yaml"

echo "🚔 Starting Enhanced Police Monitoring System with Ankaios..."
echo ""

# Check if Ankaios is installed
if ! command -v ank &> /dev/null; then
    echo "❌ Ankaios CLI (ank) not found!"
    echo "Please install Ankaios from: https://github.com/eclipse-ankaios/ankaios"
    exit 1
fi

# Check if CARLA is running
echo "🔍 Checking CARLA simulator..."
if ! nc -z localhost 2000 2>/dev/null; then
    echo "⚠️  Warning: CARLA simulator not detected on localhost:2000"
    echo "   Please start CARLA before running this system"
    read -p "   Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if container images are built
echo "🔍 Checking container images..."
REQUIRED_IMAGES=(
    "localhost/police-base:latest"
    "localhost/police-carla-spawner:latest"
    "localhost/police-kuksa-bridge:latest"
    "localhost/police-camera-bridge:latest"
    "localhost/police-websocket:latest"
)

MISSING_IMAGES=()
for image in "${REQUIRED_IMAGES[@]}"; do
    if ! podman image exists "$image"; then
        MISSING_IMAGES+=("$image")
    fi
done

if [ ${#MISSING_IMAGES[@]} -ne 0 ]; then
    echo "❌ Missing container images:"
    for image in "${MISSING_IMAGES[@]}"; do
        echo "   - $image"
    done
    echo ""
    echo "Please build the images first:"
    echo "  ./build_containers.sh"
    exit 1
fi
echo "✅ All container images found"
echo ""

# Start Ankaios server if not running
echo "🚀 Starting Ankaios server..."
if pgrep -x "ank-server" > /dev/null; then
    echo "ℹ️  Ankaios server already running"
else
    # Start server in background with insecure mode for local development
    ank-server --insecure > /tmp/ankaios-server.log 2>&1 &
    SERVER_PID=$!
    echo "✅ Ankaios server started (PID: $SERVER_PID)"

    # Wait for server to be ready
    echo "⏳ Waiting for server to be ready..."
    for i in {1..10}; do
        if ank -k get workloads > /dev/null 2>&1; then
            echo "✅ Server is ready"
            break
        fi
        if [ $i -eq 10 ]; then
            echo "❌ Server failed to start. Check logs:"
            echo "   tail -f /tmp/ankaios-server.log"
            exit 1
        fi
        sleep 1
    done
fi
echo ""

# Start Ankaios agent if not running
echo "🤖 Starting Ankaios agent (police_agent)..."
if pgrep -x "ank-agent" > /dev/null; then
    echo "ℹ️  Ankaios agent already running"
else
    # Start agent in background with insecure mode
    ank-agent --name police_agent --insecure > /tmp/ankaios-agent.log 2>&1 &
    AGENT_PID=$!
    echo "✅ Ankaios agent started (PID: $AGENT_PID)"

    # Wait for agent to connect
    echo "⏳ Waiting for agent to connect..."
    sleep 3
fi
echo ""

# Apply the manifest
echo "📋 Applying workload manifest..."
if [ ! -f "$MANIFEST_FILE" ]; then
    echo "❌ Manifest file not found: $MANIFEST_FILE"
    exit 1
fi

ank -k apply "$MANIFEST_FILE"
echo "✅ Workload manifest applied successfully"
echo ""

# Wait for workloads to start
echo "⏳ Waiting for workloads to initialize..."
sleep 5

# Show system status
echo ""
echo "📊 System Status:"
ank -k get workloads
echo ""

echo "🎉 Enhanced Police Monitoring System started successfully!"
echo ""
echo "📡 Access Points:"
echo "   • Camera Stream:     ws://localhost:8080"
echo "   • Police VSS Data:   ws://localhost:8081"
echo "   • Dashboard:         file://$(pwd)/police_dashboard_enhanced.html"
echo ""
echo "🔧 Management Commands:"
echo "   • View workloads:    ank -k get workloads"
echo "   • View logs:         podman logs <container-name>"
echo "   • Stop system:       ./stop_ankaios.sh"
echo ""
echo "Press Ctrl+C to view logs (system will continue running in background)"

# Trap Ctrl+C to show how to stop
trap 'echo ""; echo "ℹ️  System is still running. Use ./stop_ankaios.sh to stop."; exit 0' INT

# Keep script running to show it's monitoring
while true; do
    sleep 10
done
