import asyncio
import websockets
import numpy as np
import cv2
import base64  # <-- Add this

cap = cv2.VideoCapture(0)

async def send_frames(websocket):
    try:
        while True:
            ret, frame = cap.read()
            if ret:
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                # Encode as base64 string
                frame_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')

                # Send base64 string
                await websocket.send(frame_b64)

            await asyncio.sleep(0.01)  # Small delay to avoid overload
    except Exception as e:
        print(f"Connection error: {e}")

async def main():
    server_ip = "0.0.0.0"
    port = 8761
    print(f"Starting WebSocket Server on ws://{server_ip}:{port}")
    async with websockets.serve(send_frames, server_ip, port):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
