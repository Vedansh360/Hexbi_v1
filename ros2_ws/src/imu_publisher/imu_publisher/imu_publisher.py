import rclpy
from rclpy.node import Node
import serial
import json
import redis
from builtin_interfaces.msg import Time
from rclpy.clock import Clock

class IMUToRedis(Node):
    def __init__(self):
        super().__init__('imu_to_redis')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Connect to local Redis
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

        # Timer for 30 Hz publishing
        self.timer = self.create_timer(1.0 / 100.0, self.read_and_publish)

        # Frame ID to identify the sensor source
        self.frame_id = "imu_link"

        # Clock for timestamps
        self.clock = Clock()

    def read_and_publish(self):
        line = self.ser.readline().decode('utf-8').strip()
        if not line.startswith("IMU:"):
            return

        try:
            parts = line.replace("IMU:", "").split(",")
            if len(parts) != 10:
                return

            # Get ROS timestamp
            stamp = self.clock.now().to_msg()

            imu_data = {
                "header": {
                    "stamp": {
                        "sec": stamp.sec,
                        "nanosec": stamp.nanosec
                    },
                    "frame_id": self.frame_id
                },
                "orientation": {
                    "x": float(parts[1]),
                    "y": float(parts[2]),
                    "z": float(parts[3]),
                    "w": float(parts[0])
                },
                "orientation_covariance": [0.0]*9,  # or update if known
                "angular_velocity": {
                    "x": float(parts[4]),
                    "y": float(parts[5]),
                    "z": float(parts[6])
                },
                "angular_velocity_covariance": [0.0]*9,
                "linear_acceleration": {
                    "x": float(parts[7]),
                    "y": float(parts[8]),
                    "z": float(parts[9])
                },
                "linear_acceleration_covariance": [0.0]*9
            }

            # Publish JSON string to Redis
            self.redis_client.set("imu_data", json.dumps(imu_data))

        except ValueError:
            self.get_logger().warn("Invalid IMU data format")

def main(args=None):
    rclpy.init(args=args)
    node = IMUToRedis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
