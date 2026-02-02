#!/bin/bash
# Navigate to your project folder
cd /home/argus/Downloads/sentry-git/

# Pull the latest code (assumes you have saved credentials or use SSH)
git pull origin main

# Run your Python script
/home/argus/Downloads/sentry-git/env/bin/python3 server.py
