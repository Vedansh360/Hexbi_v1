import asyncio
import websockets
import numpy as np
import cv2

async def receive_frames(websocket):
    try:
        while True:
            # Receive frame data as bytes
            frame_bytes = await websocket.recv()
            frame = np.frombuffer(frame_bytes, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed normally.")
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        cv2.destroyAllWindows()

async def main():
    print("Starting WebSocket Server on ws://192.168.137.1:8761")
    try:
        async with websockets.serve(receive_frames, "192.168.137.1", 8761):
            await asyncio.Future()  # Keep running
    except asyncio.CancelledError:
        print("\nServer closed by user.")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down server...")
