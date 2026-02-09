# save as dev/map/ws_client_print.py
import asyncio, json, websockets
async def main():
    uri = "ws://100.112.223.17:8764"
    async with websockets.connect(uri) as ws:
        async for msg in ws:
            try:
                data = json.loads(msg)
                print("yaw:", data.get("yaw"))
            except:
                pass
asyncio.run(main())