#!/bin/bash
# Install Eclipse Ankaios for Police Monitoring System

set -e

ANKAIOS_VERSION="0.6.0"
INSTALL_DIR="/usr/local/bin"

echo "📦 Installing Eclipse Ankaios v${ANKAIOS_VERSION}..."
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# Create temporary directory
TMP_DIR=$(mktemp -d)
cd "$TMP_DIR"

echo "⬇️  Downloading Ankaios binaries..."

# Download tarball
TARBALL="ankaios-linux-amd64.tar.gz"
if ! wget "https://github.com/eclipse-ankaios/ankaios/releases/download/v${ANKAIOS_VERSION}/${TARBALL}"; then
    echo "❌ Failed to download Ankaios tarball"
    echo "Please check if version ${ANKAIOS_VERSION} exists at:"
    echo "https://github.com/eclipse-ankaios/ankaios/releases"
    exit 1
fi

echo "✅ Downloaded successfully"

# Extract tarball
echo "📦 Extracting binaries..."
tar -xzf "${TARBALL}"

# Make executable
chmod +x ank ank-server ank-agent

# Install binaries
echo "📥 Installing binaries to ${INSTALL_DIR}..."
$SUDO mv ank "${INSTALL_DIR}/ank"
$SUDO mv ank-server "${INSTALL_DIR}/ank-server"
$SUDO mv ank-agent "${INSTALL_DIR}/ank-agent"

echo "✅ Binaries installed"

# Clean up
cd -
rm -rf "$TMP_DIR"

# Verify installation
echo ""
echo "🔍 Verifying installation..."
ank --version
ank-server --version
ank-agent --version

echo ""
echo "🎉 Eclipse Ankaios v${ANKAIOS_VERSION} installed successfully!"
echo ""
echo "Next steps:"
echo "  1. Build container images: ./build_containers.sh"
echo "  2. Start CARLA simulator"
echo "  3. Start system: ./start_ankaios.sh"
