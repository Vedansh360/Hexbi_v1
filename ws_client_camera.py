import asyncio
import websockets
import numpy as np
import cv2
import redis
import base64

async def receive_frames():
    uri = "ws://100.113.195.60:8761"
    r = redis.Redis(host='localhost', port=6379, db=0)

    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to server at {uri}")
            while True:
                frame_bytes = await websocket.recv()
                frame = np.frombuffer(frame_bytes, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Publish to Redis
                    _, buffer = cv2.imencode('.jpg', frame)
                    encoded = base64.b64encode(buffer).decode('utf-8')
                    r.publish('camera_frame', encoded)

                    # (Optional: display it too)
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
