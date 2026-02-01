#!/bin/bash
# Quick run script for Sentry Server

# Navigate to script directory
cd "$(dirname "$0")"

# Activate virtual environment
source env/bin/activate

# Run server
echo "Starting Sentry Server..."
python server.py
