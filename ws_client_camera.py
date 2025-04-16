import asyncio
import websockets
import numpy as np
import cv2
import redis
import base64

async def receive_frames():
    uri = "ws://100.113.195.60:8761"
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to server at {uri}")
            while True:
                frame_b64 = await websocket.recv()  # receive as base64 string
                frame_bytes = base64.b64decode(frame_b64)
                frame_np = np.frombuffer(frame_bytes, dtype=np.uint8)
                frame = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)

                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame)
                    encoded = base64.b64encode(buffer).decode('utf-8')
                    redis_client.set('camera_frame', encoded.encode('utf-8'))

                    cv2.imshow("Camera Feed", frame)
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
