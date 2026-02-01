#!/bin/bash
# Quick run script for Sentry Client (Ground Station)

# Navigate to script directory
cd "$(dirname "$0")"

# Activate virtual environment
source env/bin/activate

# Run client
echo "Starting Sentry Ground Station..."
echo "Open http://localhost:8000 in your browser"
echo ""
python client.py
