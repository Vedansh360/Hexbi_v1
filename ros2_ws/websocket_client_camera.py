import asyncio
import websockets
import cv2

async def send_frames():
    uri = "ws://192.168.137.1:8761"  # Replace with your laptop's local IP
    cap = cv2.VideoCapture(0)  # Open camera

    try:
        async with websockets.connect(uri) as websocket:
            while True:
                ret, frame = cap.read()
                if ret:
                    # Encode frame as JPEG
                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()

                    # Send frame data
                    await websocket.send(frame_bytes)

                await asyncio.sleep(0.01)  # Small delay to avoid overload

    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed normally.")
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        print("Releasing camera...")
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        asyncio.run(send_frames())
    except asyncio.CancelledError:
        print("\nClient stopped by user.")
    except KeyboardInterrupt:
        print("\nShutting down client...")

