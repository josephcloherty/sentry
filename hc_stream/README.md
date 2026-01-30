# Low-Latency H.264 Video Streaming with Telemetry

This implementation provides separate client-server architecture for low-latency video streaming from Raspberry Pi to desktop client.

## Architecture

- **Server (host.py)**: Runs on Raspberry Pi
  - Streams raw H.264 video over TCP (port 8000)
  - Broadcasts telemetry via WebSocket (port 8001) at 10Hz
  - Hardware-accelerated H.264 encoding with minimal buffering
  
- **Client (client.py)**: Runs on desktop computer
  - Receives H.264 stream and decodes with PyAV
  - Receives telemetry via python-socketio
  - Renders all overlays locally (compass, attitude, telemetry text)

## Expected Latency

- **40-80ms** end-to-end (vs 150-250ms with MJPEG in stream.py)
- H.264 TCP: 30-50ms
- Telemetry WebSocket: 10-30ms

## Requirements

### Server (Raspberry Pi)
```bash
pip install picamera2 pymavlink python-socketio aiohttp psutil
```

### Client (Desktop)
```bash
pip install opencv-python numpy av python-socketio
```

## Usage

### On Raspberry Pi:
```bash
cd hc_stream
python3 host.py
```

### On Desktop Client:
```bash
cd hc_stream
python3 client.py --host <raspberry_pi_ip>
```

Example:
```bash
python3 client.py --host 192.168.1.100
```

## Configuration

Edit the constants at the top of each file:

**host.py:**
- `VIDEO_PORT = 8000` - TCP port for H.264 stream
- `TELEMETRY_PORT = 8001` - WebSocket port for telemetry
- `VIDEO_WIDTH = 640` - Video width
- `VIDEO_HEIGHT = 480` - Video height
- `VIDEO_FPS = 30` - Frame rate
- `VIDEO_BITRATE = 2000000` - H.264 bitrate (2 Mbps)
- `TELEMETRY_RATE = 10` - Telemetry broadcast rate (Hz)

**client.py:**
- `DEFAULT_HOST = '192.168.1.100'` - Default Raspberry Pi IP
- `VIDEO_PORT = 8000` - Must match server
- `TELEMETRY_PORT = 8001` - Must match server

## Client Controls

- **'o'** - Toggle overlays on/off
- **'q'** - Quit application

## Features

### Server (host.py)
- ✅ Picamera2 with low-latency configuration (buffer_count=2, queue=False)
- ✅ H.264 baseline profile for fastest decode
- ✅ TCP_NODELAY for minimal network latency
- ✅ MAVLink telemetry collection (all messages)
- ✅ CPU temperature and usage monitoring
- ✅ Asynchronous dual-server architecture (asyncio)
- ✅ Automatic MAVLink stream request at 10Hz

### Client (client.py)
- ✅ PyAV H.264 decoder with low_delay flags
- ✅ Single-threaded decode for minimal latency
- ✅ WebSocket telemetry receiver (separate thread)
- ✅ All overlay rendering functions:
  - Compass (bottom-left)
  - Attitude indicator (bottom-right)
  - Telemetry text (top-right)
  - CPU stats (top-left)
  - Latency tracking
- ✅ FPS counter
- ✅ Keyboard controls for overlay toggle

## Troubleshooting

### Video stream won't connect
- Check firewall settings on Raspberry Pi (ports 8000, 8001)
- Verify IP address is correct
- Test network connectivity: `ping <raspberry_pi_ip>`

### Telemetry shows zeros
- Check MAVLink connection on Raspberry Pi
- Verify mavlink-router is running: `systemctl status mavlink-router`
- Check mavlink-router configuration: `/etc/mavlink-router/main.conf`

### High latency or stuttering
- Reduce `VIDEO_BITRATE` in host.py (try 1000000 for 1 Mbps)
- Check network bandwidth: `iperf3 -s` on Pi, `iperf3 -c <pi_ip>` on client
- Ensure Raspberry Pi is not thermally throttling (check `vcgencmd measure_temp`)

### Overlays not rendering
- Verify all dependencies installed on client: `pip list | grep -E "opencv|numpy|av"`
- Check for Python errors in terminal output

## Differences from stream.py

This implementation differs from the original stream.py:

1. **Separate processes**: Server and client run on different machines
2. **H.264 encoding**: Hardware-accelerated, much lower bandwidth than MJPEG
3. **Raw TCP streaming**: Direct socket connection, no HTTP/Flask overhead on video path
4. **Client-side rendering**: All overlays drawn on desktop, freeing Pi CPU
5. **Dual protocol**: Video over TCP, telemetry over WebSocket (optimized for each)
6. **Lower latency**: 40-80ms vs 150-250ms with MJPEG
7. **Better bandwidth**: ~2 Mbps vs ~15+ Mbps with MJPEG

## Notes

- The original stream.py is **not modified** and can still be used independently
- MAVLink connection is optional - system will work without it (telemetry shows zeros)
- Client can be run multiple times (multiple viewers) for telemetry, but video stream is 1:1
- For production use, consider adding authentication and encryption (SSL/TLS)
