import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import redis
import time

class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')

        # Create a ROS 2 publisher
        self.publisher = self.create_publisher(String, 'direction_cmd', 10)

        # Connect to Redis
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

        # Start reading from Redis
        self.timer = self.create_timer(0.1, self.publish_from_redis)

    def publish_from_redis(self):
        """Reads input from Redis and publishes it to the direction_cmd topic."""
        message = self.redis_client.get('keyboard_input')
        
        if message:
            message = message.decode()  # Convert from bytes to string
            self.get_logger().info(f"Publishing: {message}")

            msg = String()
            msg.data = message
            self.publisher.publish(msg)

            # Clear the Redis key after publishing
            self.redis_client.delete('keyboard_input')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
