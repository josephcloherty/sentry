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

## Core ML export for Apple Silicon (M1/M2/M3)

If you want better local inference performance on a MacBook Air M2, export your YOLO `.pt` weights to Core ML `.mlpackage`:

1. Activate your project environment.
2. Run `object_detection/6_export_coreml.py` to export your active models.
3. Start the client normally.

The client will automatically prefer Core ML models on macOS when these files exist:

- `./object_detection/coreml/best8np2.mlpackage`
- `./object_detection/coreml/best26p2.mlpackage`

You can adjust all model path settings at the top of [client.py](client.py).

## Ports

- **8765** - Camera 0 video stream (WebSocket)
- **8766** - Camera 1 video stream (WebSocket)
- **8764** - Telemetry stream (WebSocket, 50Hz)
- **5000** - Legacy telemetry (UDP broadcast, 10Hz)
- **8000** - Ground station web interface (HTTP)