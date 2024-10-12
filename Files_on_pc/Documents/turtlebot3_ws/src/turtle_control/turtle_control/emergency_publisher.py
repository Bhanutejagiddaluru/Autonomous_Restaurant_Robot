#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EmergencyPublisher(Node):
    def __init__(self):
        super().__init__('emergency_publisher')
        # Initialize the publisher for the emergency topic
        self.publisher = self.create_publisher(String, '/emergency', 10)
        
        # Timer to periodically send the emergency signal
        self.timer = self.create_timer(5, self.publish_emergency_signal)  # Adjust the timer as needed

    def publish_emergency_signal(self):
        msg = String()
        msg.data = "ee"  # The emergency signal
        self.publisher.publish(msg)
        self.get_logger().info(f'Published emergency signal: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    emergency_publisher = EmergencyPublisher()
    rclpy.spin(emergency_publisher)  # Keep the node running to listen and publish messages
    rclpy.shutdown()

if __name__ == '__main__':
    main()

