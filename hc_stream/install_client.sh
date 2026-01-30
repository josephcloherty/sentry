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

echo "Installing ffmpeg..."
# Check OS and install ffmpeg accordingly
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    if command -v brew &> /dev/null; then
        brew install ffmpeg
    else
        echo "Homebrew not found. Please install Homebrew first: https://brew.sh"
        exit 1
    fi
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    sudo apt update
    sudo apt install -y ffmpeg
else
    echo "Unsupported OS. Please install ffmpeg manually."
fi

echo ""
echo "Installing Python packages..."
pip3 install opencv-python numpy python-socketio

echo ""
echo "Installation complete!"
echo ""
echo "To run the client:"
echo "  python3 client.py --host <raspberry_pi_ip>"
echo ""
echo "Example:"
echo "  python3 client.py --host 192.168.1.100"
