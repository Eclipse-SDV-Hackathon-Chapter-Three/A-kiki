#!/bin/bash
# Build all container images for Ankaios-managed Police Monitoring System

set -e

echo "🐳 Building Police Monitoring System Containers..."
echo ""

# Build base image first
echo "📦 Building base image..."
podman build --no-cache -t localhost/police-base:latest -f Dockerfile.base .
echo "✅ Base image built successfully"
echo ""

# Build CARLA spawner
echo "🚗 Building CARLA spawner image..."
podman build --no-cache -t localhost/police-carla-spawner:latest -f Dockerfile.carla-spawner .
echo "✅ CARLA spawner image built successfully"
echo ""

# Build KUKSA bridge
echo "🌉 Building KUKSA-zenoh bridge image..."
podman build --no-cache -t localhost/police-kuksa-bridge:latest -f Dockerfile.kuksa-bridge .
echo "✅ KUKSA-zenoh bridge image built successfully"
echo ""

# Build camera bridge
echo "📸 Building camera zenoh bridge image..."
podman build --no-cache -t localhost/police-camera-bridge:latest -f Dockerfile.camera-bridge .
echo "✅ Camera zenoh bridge image built successfully"
echo ""

# Build WebSocket bridges
echo "🌐 Building WebSocket bridges image..."
podman build --no-cache -t localhost/police-websocket:latest -f Dockerfile.websocket .
echo "✅ WebSocket bridges image built successfully"
echo ""

echo "🎉 All container images built successfully!"
echo ""
echo "Available images:"
podman images | grep "localhost/police"
echo ""
echo "You can now start the Ankaios system with:"
echo "  ./start_ankaios.sh"
