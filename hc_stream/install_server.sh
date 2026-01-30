#!/bin/bash
# Install dependencies for low-latency streaming server
# Run this on your Raspberry Pi

echo "Installing server dependencies..."
echo "================================="
echo ""

# Check if pip is available
if ! command -v pip3 &> /dev/null; then
    echo "Error: pip3 not found. Please install Python 3 first."
    exit 1
fi

echo "Installing Python packages..."
pip3 install picamera2 pymavlink python-socketio aiohttp psutil

echo ""
echo "Installation complete!"
echo ""
echo "To run the server:"
echo "  python3 host.py"
echo ""
echo "Note: Make sure mavlink-router is running for telemetry data"
