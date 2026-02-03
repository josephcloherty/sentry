#!/usr/bin/env python3
"""
Simple WebSocket telemetry publisher for testing the map client.
Sends JSON messages like: {"yaw": 123.45, "timestamp": 167...}
Listens on port 8764 by default (matches your server/client code).
"""
import asyncio
import json
import math
import time

import websockets

PORT = 5000
CLIENTS = set()

async def producer():
    """Broadcast a synthetic yaw value to all connected clients at 20Hz."""
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            # Synthetic yaw: slowly rotating value (0-360) with small oscillation
            yaw = (t * 30) % 360  # 30 deg/sec rotation
            yaw += 10.0 * math.sin(t * 2.0)  # small oscillation
            payload = json.dumps({"yaw": round(yaw, 2), "timestamp": time.time()})
            if CLIENTS:
                await asyncio.gather(*(ws.send(payload) for ws in set(CLIENTS)))
            await asyncio.sleep(1.0 / 20.0)
    except asyncio.CancelledError:
        return

async def handler(ws):
    print('Telemetry test client connected')
    CLIENTS.add(ws)
    try:
        await ws.wait_closed()
    finally:
        CLIENTS.discard(ws)
        print('Telemetry test client disconnected')

async def main():
    # websockets >=11 calls the handler with a single `websocket` argument,
    # so pass the `handler` directly (it accepts one argument).
    server = await websockets.serve(handler, '0.0.0.0', PORT)
    print(f'Telemetry test publisher running on ws://0.0.0.0:{PORT}')
    producer_task = asyncio.create_task(producer())
    try:
        await asyncio.Future()  # run forever
    finally:
        producer_task.cancel()
        server.close()
        await server.wait_closed()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\nStopped by user')
