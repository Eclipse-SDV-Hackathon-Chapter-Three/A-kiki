#!/bin/bash
# Stop Ankaios-managed Police Monitoring System

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🛑 Stopping Enhanced Police Monitoring System..."
echo ""

# Check if Ankaios CLI is available
if ! command -v ank &> /dev/null; then
    echo "❌ Ankaios CLI (ank) not found!"
    exit 1
fi

# Delete all workloads
echo "🗑️  Deleting workloads..."
WORKLOADS=(
    "zenoh-router"
    "carla-vss-spawner"
    "kuksa-zenoh-bridge"
    "camera-zenoh-bridge"
    "camera-websocket"
    "police-websocket"
)

for workload in "${WORKLOADS[@]}"; do
    if ank -k get workloads | grep -q "$workload"; then
        echo "   Stopping $workload..."
        ank -k delete workload "$workload" 2>/dev/null || true
    fi
done

echo "✅ All workloads deleted"
echo ""

# Wait for containers to stop
echo "⏳ Waiting for containers to stop..."
sleep 3

# Force stop any remaining containers
echo "🧹 Cleaning up containers..."
podman ps -a --filter "name=zenoh-router" -q | xargs -r podman rm -f 2>/dev/null || true
podman ps -a --filter "name=carla-vss-spawner" -q | xargs -r podman rm -f 2>/dev/null || true
podman ps -a --filter "name=kuksa-zenoh-bridge" -q | xargs -r podman rm -f 2>/dev/null || true
podman ps -a --filter "name=camera-zenoh-bridge" -q | xargs -r podman rm -f 2>/dev/null || true
podman ps -a --filter "name=camera-websocket" -q | xargs -r podman rm -f 2>/dev/null || true
podman ps -a --filter "name=police-websocket" -q | xargs -r podman rm -f 2>/dev/null || true

echo "✅ Containers cleaned up"
echo ""

# Optionally stop Ankaios server and agent
read -p "Stop Ankaios server and agent? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🛑 Stopping Ankaios components..."

    if pgrep -x "ank-agent" > /dev/null; then
        pkill -TERM ank-agent
        echo "✅ Ankaios agent stopped"
    fi

    if pgrep -x "ank-server" > /dev/null; then
        pkill -TERM ank-server
        echo "✅ Ankaios server stopped"
    fi
fi

echo ""
echo "🎉 Enhanced Police Monitoring System stopped successfully!"
