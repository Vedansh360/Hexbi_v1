import asyncio
import websockets
import redis

# Connect to Redis
redis_client = redis.Redis(host='localhost', port=6379, db=0)

async def websocket_handler(websocket):
    try:
        print("Connected to motor control client.")
        async for message in websocket:
            print(f"Received from client: {message}")

            # Store the message in Redis for the ROS 2 node
            redis_client.set('keyboard_input', message)

            # Acknowledge reception
            await websocket.send("Message received")
    except Exception as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("Shutting Down Motor Control Server.")

async def start_server():
    """Starts the WebSocket server."""
    server = await websockets.serve(websocket_handler, "0.0.0.0", 8765, ping_interval=None)
    print("WebSocket server started on ws://0.0.0.0:8765")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(start_server())
