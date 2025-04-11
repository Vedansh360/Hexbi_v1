import asyncio
import websockets

async def receive_data(websocket):
    async for message in websocket:
        print(f"Received: {message}")
        response = f"Echo: {message}"
        await websocket.send(response)  # Echo back the message

async def main():
    print("Starting WebSocket Server on ws://0.0.0.0:8765")
    async with websockets.serve(receive_data, "0.0.0.0", 8765):
        await asyncio.Future()  # Keep running

asyncio.run(main())
