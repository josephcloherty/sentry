# Sentry UAS Dashboard

Ground station dashboard for the S.E.N.T.R.Y UAS with real-time telemetry, dual video feeds, and GPS tracking.

## Quick Start

### Server (Raspberry Pi)
```bash
./install_server.sh  # Install packages and dependencies
./run_server.sh      # Start server
```

### Client (Ground Station)
```bash
./install_client.sh  # Install packages and dependencies
./run_client.sh      # Start dashboard
```

Open http://localhost:8000 in a browser

## Ports

- **8765** - Camera 0 video stream (WebSocket)
- **8766** - Camera 1 video stream (WebSocket)
- **8764** - Telemetry stream (WebSocket, 50Hz)
- **5000** - Legacy telemetry (UDP broadcast, 10Hz)
- **8000** - Ground station web interface (HTTP)