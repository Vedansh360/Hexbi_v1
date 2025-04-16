import rclpy
from rclpy.node import Node
import redis
import base64
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class RedisImagePublisher(Node):
    def __init__(self):
        super().__init__('redis_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        self.thread = threading.Thread(target=self.listen_to_redis)
        self.thread.daemon = True
        self.thread.start()

    def listen_to_redis(self):
        pubsub = self.redis_client.pubsub()
        pubsub.subscribe('camera_frame')
        for message in pubsub.listen():
            if message['type'] == 'message':
                encoded = message['data']
                try:
                    img_data = base64.b64decode(encoded)
                    nparr = np.frombuffer(img_data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        self.publisher_.publish(ros_image)
                except Exception as e:
                    self.get_logger().error(f"Decoding error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RedisImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
