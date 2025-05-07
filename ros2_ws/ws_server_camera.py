import asyncio
import websockets
import numpy as np
import cv2
import base64
import json
from datetime import datetime

# Initialize camera with desired resolution
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

async def send_frames(websocket):
    try:
        while True:
            ret, frame = cap.read()
            if ret:
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                frame_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')

                # Get current timestamp
                timestamp = datetime.now().timestamp()

                # Create message dictionary
                message = {
                    "timestamp": timestamp,
                    "frame": frame_b64
                }

                # Send as JSON
                await websocket.send(json.dumps(message))

            await asyncio.sleep(1/10)  # Small delay
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
