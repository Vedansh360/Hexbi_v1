import asyncio
import websockets

connected_clients = set()  # Store connected WebSocket clients

async def handle_client(websocket, path):
    """ Handles new WebSocket client connections """
    connected_clients.add(websocket)
    try:
        async for _ in websocket:
            pass  # Keep the connection open
    except websockets.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)

async def send_commands():
    """ Gets keyboard input and sends it to all connected WebSocket clients """
    while True:
        cmd = input("Enter command (F/B/L/R/S): ").strip().upper()
        if cmd in ['F', 'B', 'L', 'R', 'S']:
            print(f"Sending: {cmd}")
            if connected_clients:
                await asyncio.wait([client.send(cmd) for client in connected_clients])
            else:
                print("No clients connected.")
        else:
            print("Invalid command! Please enter F, B, L, R, or S.")

async def main():
    """ Starts the WebSocket server and listens for connections """
    server = await websockets.serve(handle_client, "0.0.0.0", 8765)
    print("WebSocket Server started on ws://0.0.0.0:8765")

    await asyncio.gather(server.wait_closed(), send_commands())

if __name__ == "__main__":
    asyncio.run(main())
