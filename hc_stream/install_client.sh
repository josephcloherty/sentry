#!/bin/bash
# Install dependencies for low-latency streaming client
# Run this on your desktop/laptop computer

echo "Installing client dependencies..."
echo "================================="
echo ""

# Check if pip is available
if ! command -v pip3 &> /dev/null; then
    echo "Error: pip3 not found. Please install Python 3 first."
    exit 1
fi

echo "Installing Python packages..."
pip3 install opencv-python numpy av python-socketio

echo ""
echo "Installation complete!"
echo ""
echo "To run the client:"
echo "  python3 client.py --host <raspberry_pi_ip>"
echo ""
echo "Example:"
echo "  python3 client.py --host 192.168.1.100"
