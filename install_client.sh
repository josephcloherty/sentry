#!/bin/bash
# Sentry Client (Ground Station) Installation Script
# Run on ground station computer

set -e  # Exit on error

echo "========================================"
echo "  Sentry Client Installation Script"
echo "========================================"
echo ""

# Check if running as root (not recommended for pip)
if [ "$EUID" -eq 0 ]; then
    echo "Warning: Running as root. Consider using a virtual environment."
fi

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "Python version: $PYTHON_VERSION"

# Minimum Python version check
REQUIRED_VERSION="3.8"
if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "Error: Python $REQUIRED_VERSION or higher is required"
    exit 1
fi

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

# Install system dependencies (macOS/Linux)
echo ""
echo "Checking for system dependencies..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    if command -v brew &> /dev/null; then
        echo "Installing system dependencies via Homebrew..."
        brew install gdal proj geos 2>/dev/null || echo "Some packages may already be installed"
    fi
elif command -v apt-get &> /dev/null; then
    # Debian/Ubuntu
    echo "Installing system dependencies via apt..."
    sudo apt-get update
    sudo apt-get install -y \
        libgdal-dev \
        gdal-bin \
        libproj-dev \
        libgeos-dev \
        2>/dev/null || echo "Some packages may not be available"
fi

# Install Python packages
echo ""
echo "Installing Python packages..."
pip install -r requirements_client.txt

echo ""
echo "========================================"
echo "  Installation Complete!"
echo "========================================"
echo ""
echo "To run the client:"
echo "  source env/bin/activate"
echo "  python client.py"
echo ""
echo "Or simply run:"
echo "  ./run_client.sh"
echo ""
echo "Then open http://localhost:8000 in your browser"
echo ""
