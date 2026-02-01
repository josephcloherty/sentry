#!/bin/bash
# Sentry Server Installation Script
# Run on Raspberry Pi / vehicle computer

set -e  # Exit on error

echo "========================================"
echo "  Sentry Server Installation Script"
echo "========================================"
echo ""

# Check if running as root (not recommended for pip)
if [ "$EUID" -eq 0 ]; then
    echo "Warning: Running as root. Consider using a virtual environment."
fi

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "Python version: $PYTHON_VERSION"

# Create virtual environment if it doesn't exist
if [ ! -d "env" ]; then
    echo ""
    echo "Creating virtual environment..."
    python3 -m venv env
fi

# Activate virtual environment
echo "Activating virtual environment..."
source env/bin/activate

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip

# Install system dependencies for OpenCV (Raspberry Pi)
echo ""
echo "Checking for system dependencies..."
if command -v apt-get &> /dev/null; then
    echo "Installing system dependencies for OpenCV..."
    sudo apt-get update
    sudo apt-get install -y \
        libatlas-base-dev \
        libhdf5-dev \
        libhdf5-serial-dev \
        libjasper-dev \
        libqtgui4 \
        libqt4-test \
        libcamera-dev \
        python3-libcamera \
        python3-picamera2 \
        2>/dev/null || echo "Some packages may not be available on your system"
fi

# Install Python packages
echo ""
echo "Installing Python packages..."
pip install -r requirements_server.txt

echo ""
echo "========================================"
echo "  Installation Complete!"
echo "========================================"
echo ""
echo "To run the server:"
echo "  source env/bin/activate"
echo "  python server.py"
echo ""
echo "Or simply run:"
echo "  ./run_server.sh"
echo ""
