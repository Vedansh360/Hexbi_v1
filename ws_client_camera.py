import asyncio
import websockets
import numpy as np
import cv2
import redis
import base64
import json

async def receive_frames():
    uri = "ws://100.113.195.60:8761"
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to server at {uri}")
            while True:
                message = await websocket.recv()
                data = json.loads(message)

                # Extract frame and timestamp (you can use timestamp if needed)
                frame_b64 = data.get("frame", "")
                #print("Timestamp:", data.get("timestamp"))
                frame_bytes = base64.b64decode(frame_b64)
                frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
                frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Show the frame
                    cv2.imshow("Camera Feed", frame)

                    # Publish the entire JSON message to Redis
                    redis_client.set('camera_frame', json.dumps(data).encode('utf-8'))

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        asyncio.run(receive_frames())
    except KeyboardInterrupt:
        print("\nClient shutdown.")
