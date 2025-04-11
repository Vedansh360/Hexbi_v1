import asyncio
import websockets

async def send_data():
    uri = "ws://192.168.137.1:8765"  # Replace with your laptop's local IP

    async with websockets.connect(uri) as websocket:
        message = "Hello from Raspberry Pi!"
        print(f"Sending: {message}")
        await websocket.send(message)  # Send message

        response = await websocket.recv()  # Wait for response
        print(f"Received from server: {response}")

asyncio.run(send_data())
