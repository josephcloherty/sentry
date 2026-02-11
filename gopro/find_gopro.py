#!/usr/bin/env python3
"""
GoPro Device Finder
Scans all video devices to find the GoPro camera
"""

import cv2
import subprocess
import re

def run_command(cmd):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout
    except Exception as e:
        return f"Error: {e}"

def check_device(index):
    """Check if a video device works with OpenCV"""
    try:
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            return ret and frame is not None
        return False
    except:
        return False

print("=" * 60)
print("GoPro Device Diagnostic Tool")
print("=" * 60)

# Check USB devices
print("\n1. USB Devices (looking for GoPro):")
print("-" * 60)
usb_output = run_command("lsusb | grep -i gopro")
if usb_output.strip():
    print(usb_output)
else:
    print("⚠ No GoPro found in USB devices!")
    print("\nAll USB devices:")
    print(run_command("lsusb"))

# Check video devices
print("\n2. Video Devices:")
print("-" * 60)
v4l_output = run_command("v4l2-ctl --list-devices")
print(v4l_output)

# List all /dev/video* devices
print("\n3. Available /dev/video* devices:")
print("-" * 60)
video_devices = run_command("ls -la /dev/video* 2>/dev/null")
print(video_devices if video_devices else "No video devices found!")

# Test each video device with OpenCV
print("\n4. Testing OpenCV access to each device:")
print("-" * 60)
working_devices = []

for i in range(10):  # Test video0 through video9
    print(f"\nTesting /dev/video{i}...", end=" ")
    
    # Get device info
    info = run_command(f"v4l2-ctl -d /dev/video{i} --info 2>/dev/null")
    
    if "Failed to open" in info or not info.strip():
        print("❌ Device not found")
        continue
    
    # Extract card name
    card_match = re.search(r'Card type\s*:\s*(.+)', info)
    card_name = card_match.group(1).strip() if card_match else "Unknown"
    
    print(f"Found: {card_name}")
    
    # Check supported formats
    formats = run_command(f"v4l2-ctl -d /dev/video{i} --list-formats-ext 2>/dev/null")
    
    # Test with OpenCV
    if check_device(i):
        print(f"  ✅ OpenCV can read from this device!")
        working_devices.append((i, card_name))
        
        # Get capabilities
        caps = run_command(f"v4l2-ctl -d /dev/video{i} -D 2>/dev/null")
        if "Video Capture" in caps:
            print(f"  ✅ Device supports video capture")
            
            # Show some formats
            if "MJPEG" in formats or "YUYV" in formats or "H264" in formats:
                print(f"  ✅ Supports streaming formats")
                if "MJPEG" in formats:
                    print("     - MJPEG (Motion JPEG)")
                if "YUYV" in formats:
                    print("     - YUYV (YUV 4:2:2)")
                if "H264" in formats:
                    print("     - H264")
    else:
        print(f"  ❌ OpenCV cannot read from this device")
        
        # Try to diagnose why
        if "metadata" in card_name.lower():
            print(f"     (This is a metadata device, not video)")
        elif not "Video Capture" in run_command(f"v4l2-ctl -d /dev/video{i} -D 2>/dev/null"):
            print(f"     (Device doesn't support video capture)")

# Summary
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)

if working_devices:
    print(f"\n✅ Found {len(working_devices)} working video device(s):")
    for idx, name in working_devices:
        print(f"   /dev/video{idx} - {name}")
    
    print("\n📝 To use in your script, set:")
    print(f"   CAMERA_INDEX = {working_devices[0][0]}")
    
    # Check if it's a GoPro
    gopro_devices = [(idx, name) for idx, name in working_devices if 'gopro' in name.lower()]
    if gopro_devices:
        print(f"\n🎥 GoPro detected at /dev/video{gopro_devices[0][0]}")
    else:
        print("\n⚠ No device named 'GoPro' found. The GoPro might be labeled differently.")
        print("   Check the device names above.")
else:
    print("\n❌ No working video devices found!")
    print("\nTroubleshooting steps:")
    print("1. Make sure GoPro is connected via USB")
    print("2. Put GoPro in Webcam mode:")
    print("   - Swipe down → Preferences → Connections → USB → GoPro Connect/Webcam")
    print("3. Check if camera appears in 'lsusb'")
    print("4. Try: sudo modprobe v4l2loopback")
    print("5. Check permissions: sudo usermod -a -G video $USER")

print("\n" + "=" * 60)#!/usr/bin/env python3
"""
GoPro Device Finder
Scans all video devices to find the GoPro camera
"""

import cv2
import subprocess
import re

def run_command(cmd):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout
    except Exception as e:
        return f"Error: {e}"

def check_device(index):
    """Check if a video device works with OpenCV"""
    try:
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            return ret and frame is not None
        return False
    except:
        return False

print("=" * 60)
print("GoPro Device Diagnostic Tool")
print("=" * 60)

# Check USB devices
print("\n1. USB Devices (looking for GoPro):")
print("-" * 60)
usb_output = run_command("lsusb | grep -i gopro")
if usb_output.strip():
    print(usb_output)
else:
    print("⚠ No GoPro found in USB devices!")
    print("\nAll USB devices:")
    print(run_command("lsusb"))

# Check video devices
print("\n2. Video Devices:")
print("-" * 60)
v4l_output = run_command("v4l2-ctl --list-devices")
print(v4l_output)

# List all /dev/video* devices
print("\n3. Available /dev/video* devices:")
print("-" * 60)
video_devices = run_command("ls -la /dev/video* 2>/dev/null")
print(video_devices if video_devices else "No video devices found!")

# Test each video device with OpenCV
print("\n4. Testing OpenCV access to each device:")
print("-" * 60)
working_devices = []

for i in range(10):  # Test video0 through video9
    print(f"\nTesting /dev/video{i}...", end=" ")
    
    # Get device info
    info = run_command(f"v4l2-ctl -d /dev/video{i} --info 2>/dev/null")
    
    if "Failed to open" in info or not info.strip():
        print("❌ Device not found")
        continue
    
    # Extract card name
    card_match = re.search(r'Card type\s*:\s*(.+)', info)
    card_name = card_match.group(1).strip() if card_match else "Unknown"
    
    print(f"Found: {card_name}")
    
    # Check supported formats
    formats = run_command(f"v4l2-ctl -d /dev/video{i} --list-formats-ext 2>/dev/null")
    
    # Test with OpenCV
    if check_device(i):
        print(f"  ✅ OpenCV can read from this device!")
        working_devices.append((i, card_name))
        
        # Get capabilities
        caps = run_command(f"v4l2-ctl -d /dev/video{i} -D 2>/dev/null")
        if "Video Capture" in caps:
            print(f"  ✅ Device supports video capture")
            
            # Show some formats
            if "MJPEG" in formats or "YUYV" in formats or "H264" in formats:
                print(f"  ✅ Supports streaming formats")
                if "MJPEG" in formats:
                    print("     - MJPEG (Motion JPEG)")
                if "YUYV" in formats:
                    print("     - YUYV (YUV 4:2:2)")
                if "H264" in formats:
                    print("     - H264")
    else:
        print(f"  ❌ OpenCV cannot read from this device")
        
        # Try to diagnose why
        if "metadata" in card_name.lower():
            print(f"     (This is a metadata device, not video)")
        elif not "Video Capture" in run_command(f"v4l2-ctl -d /dev/video{i} -D 2>/dev/null"):
            print(f"     (Device doesn't support video capture)")

# Summary
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)

if working_devices:
    print(f"\n✅ Found {len(working_devices)} working video device(s):")
    for idx, name in working_devices:
        print(f"   /dev/video{idx} - {name}")
    
    print("\n📝 To use in your script, set:")
    print(f"   CAMERA_INDEX = {working_devices[0][0]}")
    
    # Check if it's a GoPro
    gopro_devices = [(idx, name) for idx, name in working_devices if 'gopro' in name.lower()]
    if gopro_devices:
        print(f"\n🎥 GoPro detected at /dev/video{gopro_devices[0][0]}")
    else:
        print("\n⚠ No device named 'GoPro' found. The GoPro might be labeled differently.")
        print("   Check the device names above.")
else:
    print("\n❌ No working video devices found!")
    print("\nTroubleshooting steps:")
    print("1. Make sure GoPro is connected via USB")
    print("2. Put GoPro in Webcam mode:")
    print("   - Swipe down → Preferences → Connections → USB → GoPro Connect/Webcam")
    print("3. Check if camera appears in 'lsusb'")
    print("4. Try: sudo modprobe v4l2loopback")
    print("5. Check permissions: sudo usermod -a -G video $USER")

print("\n" + "=" * 60)