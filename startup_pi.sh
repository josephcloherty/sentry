#!/bin/bash
# Navigate to your project folder
cd /home/argus/Downloads/sentry-git/

# Pull the latest code (assumes you have saved credentials or use SSH)
git pull origin main

#Start GoPro Webcam service
sudo gopro webcam --non-interactive --resolution 1080 --fov wide --auto-start --video-number 42

# Run your Python script
/home/argus/Downloads/sentry-git/env/bin/python3 server.py
