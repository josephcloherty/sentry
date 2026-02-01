# Sentry UAS Dashboard

Ground station dashboard for the S.E.N.T.R.Y UAS with real-time telemetry, dual video feeds, and GPS tracking.

## Quick Start

### Server (Raspberry Pi / Vehicle)
```bash
./install_server.sh  # First time only
./run_server.sh      # Start server
```

### Client (Ground Station)
```bash
./install_client.sh  # First time only
./run_client.sh      # Start dashboard
```

Open http://localhost:8000 in your browser.

## Configuration

### Client Configuration (`client.py`)

Adjust these variables at the top of `client.py`:

```python
# Update intervals (milliseconds)
MAP_UPDATE_INTERVAL_MS = 5000      # How often to regenerate map
STATUS_UPDATE_INTERVAL_MS = 2000   # How often to poll connection status
STATUS_TIMEOUT = 3.0               # Seconds before marking offline
```

### Server Configuration (`server.py`)

Adjust these variables at the top of `server.py`:

```python
VIDEO_FPS = 15              # Video frame rate
TELEMETRY_HZ = 50           # Telemetry update rate (affects latency)
JPEG_QUALITY = 75           # Video quality (0-95)
```

## Features

- ✅ Dual camera video streams (WebSocket)
- ✅ Real-time MAVLink telemetry (50Hz WebSocket + 10Hz UDP fallback)
- ✅ Live GPS tracking with auto-updating map
- ✅ Automatic connection status monitoring
- ✅ Attitude indicator, compass, and battery widgets
- ✅ Low latency (~40-60ms telemetry with WebSocket)

## Ports

- **8765** - Camera 0 video stream (WebSocket)
- **8766** - Camera 1 video stream (WebSocket)
- **8764** - Telemetry stream (WebSocket, 50Hz)
- **5000** - Legacy telemetry (UDP broadcast, 10Hz)
- **8000** - Ground station web interface (HTTP)