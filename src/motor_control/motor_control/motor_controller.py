import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            String,
            'direction_cmd',
            self.listener_callback,
            10)
        self.arduino = None
        
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust to match Arduino port
            time.sleep(2)
            self.get_logger().info("Connected to Arduino")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

    def listener_callback(self, msg):
        if self.arduino and self.arduino.is_open:
            command = msg.data
            self.get_logger().info(f"Received command: {command}")
            self.arduino.write(command.encode())
            self.get_logger().info(f"Sent command to Arduino: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

