import asyncio
import redis
import websockets

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
IMU_KEY = "imu_data"

async def handle_client(websocket):
    prev_data = None
    while True:
        imu_data = redis_client.get(IMU_KEY)
        if imu_data != prev_data:
            await websocket.send(imu_data)
            prev_data = imu_data
        await asyncio.sleep(0.01)

async def main():
    async with websockets.serve(handle_client, "0.0.0.0", 8762):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
