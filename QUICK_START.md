# Quick Start Guide - Running the Improved System

## Prerequisites

Ensure these packages are installed:
```bash
pip install websockets asyncio cv2 pymavlink flask
```

## Starting the System

### 1. Start the Server (on vehicle/host with cameras)

```bash
cd /Users/josephcloherty/Desktop/AERO420/sentry-git
python3 server.py
```

**Expected output:**
```
Connecting to MAVLink...
Awaiting MAVLink heartbeat...
MAVLink heartbeat received.
Server running and ready for connections on ports 8765 (cam0), 8766 (cam1), 8764 (telemetry).
[Telemetry] Broadcasting at 50Hz, avg server latency: X.XXms, WebSocket clients: 0
```

### 2. Start the Client (on ground station/operator machine)

In a new terminal:
```bash
cd /Users/josephcloherty/Desktop/AERO420/sentry-git
python3 client.py
```

**Expected output:**
```
Connected to video server (cam0).
Connected to video server (cam1).
Connected to telemetry server (WebSocket).
 * Serving Flask app 'client'
 * Running on http://127.0.0.1:8000
```

### 3. Open Dashboard in Browser

Navigate to: `http://localhost:8000/`

---

## Monitoring Telemetry Latency

### In Browser Console (F12)

```javascript
// Monitor latency in real-time
// Press F12 to open Developer Tools → Console
// You'll see output like:
// [Telemetry] Latency: 45.2ms (client: 32.1ms + server: 13.1ms)
```

### Via `/telemetry` Endpoint

```bash
curl http://localhost:8000/telemetry | python3 -m json.tool
```

**Output example:**
```json
{
  "roll": 0.123,
  "pitch": -1.456,
  "yaw": 180.0,
  "lat": 53.406049,
  "lon": -2.968585,
  "alt": 45.2,
  "battery": 14.8,
  "battery_remaining": 85,
  "ground_speed": 12.5,
  "throttle": 65,
  "timestamp": 1706814234.5678,
  "server_latency_ms": 1.23,
  "client_latency_ms": 45.67
}
```

### In Server Output

Watch the server terminal for latency reports every ~2 seconds:
```
[Telemetry] Broadcasting at 50Hz, avg server latency: 1.23ms, WebSocket clients: 1
```

---

## Architecture Overview

```
VEHICLE/HOST                          GROUND STATION/OPERATOR
┌──────────────────────┐             ┌──────────────────────┐
│   Autopilot (MAV)    │             │      Browser         │
│   (MAVLink stream)   │             │    (Dashboard UI)    │
└──────────┬───────────┘             └──────────┬───────────┘
           │                                    ▲
           │ MAVLink (UDP 14550)                │
           │                                    │ HTTP
           ▼                                    │
┌──────────────────────┐             ┌──────────┴───────────┐
│   server.py          │             │    client.py         │
│  ┌────────────────┐  │             │  ┌─────────────────┐ │
│  │ Telemetry WS   │  │ ────────┐   │  │ Flask Server    │ │
│  │ (port 8764)    │  │         │   │  │ (port 8000)     │ │
│  └────────────────┘  │         │   │  └─────────────────┘ │
│                      │         │   │  ┌─────────────────┐ │
│  ┌────────────────┐  │         └──►│  │ WebSocket Client│ │
│  │ Video Streams  │  │             │  │ (telemetry)     │ │
│  │ (cam0/cam1)    │  │ ───────────►│  │                 │ │
│  │ (8765/8766)    │  │             │  └─────────────────┘ │
│  └────────────────┘  │             │                      │
│                      │             │                      │
└──────────────────────┘             └──────────────────────┘
```

---

## Key Improvements vs Before

| Aspect | Before | After |
|--------|--------|-------|
| **Telemetry Latency** | ~10 seconds | ~100-200ms |
| **Update Frequency** | 10Hz (100ms) | 50Hz (20ms) |
| **Protocol** | UDP broadcast | WebSocket (reliable) |
| **Browser Update** | HTTP polling (1-5s) | WebSocket push (real-time) |
| **Diagnostics** | None | Latency tracking in every packet |
| **Network Efficiency** | Broadcast to all | Point-to-point to connected clients |

---

## Troubleshooting

### Telemetry Not Updating?

1. **Check server is running:**
   ```bash
   netstat -an | grep 8764
   # Should see LISTEN on port 8764
   ```

2. **Check WebSocket connection in browser:**
   ```javascript
   // In browser console:
   console.log(telemetryData);
   // Should show current telemetry object
   ```

3. **Check client is connecting to server:**
   - Look for "Connected to telemetry server (WebSocket)." in client.py output

### High Latency Still Observed?

1. **Check server-side latency:**
   - Look at server output: `avg server latency: X.XXms`
   - Should be <5ms typically

2. **Check network latency:**
   - View browser console for client latency: `[Telemetry] Latency: Xms`
   - If >100ms, may be network issue

3. **Check if UDP fallback is active:**
   - Server should show "WebSocket clients: 1" (or more)
   - If "WebSocket clients: 0", browser isn't connected to telemetry

### Connection Dropped?

The system will automatically reconnect:
1. Browser: Reconnects every 2 seconds
2. Client: Reconnects every 2 seconds
3. Check console for messages about reconnection attempts

---

## Performance Expectations

### Typical Latency Breakdown
```
MAVLink message generated on autopilot     0ms
Transmitted to server (MAVLink UDP)        +5ms
Server processing & broadcast              +1ms  (server_latency_ms)
Network transit to client                  +20ms
Client processing & browser send           +5ms
Browser receives & displays                +5ms
─────────────────────────────────────────
TOTAL LATENCY                              ~36ms
```

With multiple clients or congestion, could be 50-200ms.

---

## System Health Indicators

### ✅ Good Health
- Server shows `avg server latency: <5ms`
- Browser console shows latency 20-100ms
- Video feeds update smoothly
- Telemetry values change in real-time

### ⚠️ Warning Signs
- Server latency >20ms (CPU overloaded)
- Browser latency >500ms (network congestion)
- Long pauses between telemetry updates
- WebSocket clients showing "0" in server output

### ❌ Problem State
- Server not broadcasting (check MAVLink connection)
- Browser console shows "disconnected" messages
- No updates to telemetry for >5 seconds
- High packet loss (UDP fallback may be triggering)

---

## Next Steps

1. **Monitor the system** - Watch latency in browser console for 5 minutes
2. **Record baseline** - Note typical latency under normal conditions
3. **Test under load** - Open multiple browser tabs, see how latency scales
4. **Optimize if needed** - See TELEMETRY_LATENCY_ANALYSIS.md for further improvements

---

## References

- [TELEMETRY_LATENCY_ANALYSIS.md](TELEMETRY_LATENCY_ANALYSIS.md) - Detailed technical analysis
- [IMPLEMENTATION_COMPLETE.md](IMPLEMENTATION_COMPLETE.md) - Implementation details
- server.py - Telemetry server with WebSocket
- client.py - Dashboard with WebSocket client
- templates/index.html - Browser interface with real-time updates
