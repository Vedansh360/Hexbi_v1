import asyncio
import websockets

async def send_command():
    uri = "ws://192.168.137.120:8765"  # Replace with your ROS 2 WebSocket server IP
    
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print("✅ Connected to WebSocket server.")
                
                while True:
                    cmd = await asyncio.to_thread(input, "Enter command (wasd|b|q): ")
                    cmd = cmd.strip().lower()

                    if cmd in ['w', 'b', 'd', 'a', 's']:
                        await websocket.send(cmd)
                        print(f"📤 Sent: {cmd}")
                    elif cmd == "q":
                        print("🔴 Exiting...")
                        return
                    else:
                        print("⚠️ Invalid command! Enter w,a,s,d,b or q to quit.")
        
        except (websockets.exceptions.ConnectionClosedError, websockets.exceptions.ConnectionClosedOK):
            print("⚠️ Connection lost. Reconnecting in 3 seconds...")
            await asyncio.sleep(3)
        except KeyboardInterrupt:
            print("🔴 Closing connection...")
            return

# Run the async function
asyncio.run(send_command())
