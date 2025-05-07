import asyncio
import websockets
import json
import redis

# Connect to Redis
redis_client = redis.Redis(host='localhost', port=6379, db=0)

async def listen():
    uri = "ws://100.113.195.60:8762"
    async with websockets.connect(uri) as websocket:
        while True:
            imu_data = await websocket.recv()
            try:
                imu_json = json.loads(imu_data)
                # Publish JSON string to Redis under the key 'imu_data'
                redis_client.set('imu_data', json.dumps(imu_json))
            except json.JSONDecodeError:
                print("Error parsing JSON:", imu_data)

if __name__ == "__main__":
    asyncio.run(listen())
