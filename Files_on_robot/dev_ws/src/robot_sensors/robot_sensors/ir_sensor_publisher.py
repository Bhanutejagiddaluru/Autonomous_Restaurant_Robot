import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class IRSensorPublisher(Node):
    def __init__(self):
        super().__init__('ir_sensor_publisher')
        GPIO.setmode(GPIO.BCM)
        self.ir_pin_front = 24
        self.ir_pin_back = 23
        GPIO.setup(self.ir_pin_front, GPIO.IN)
        GPIO.setup(self.ir_pin_back, GPIO.IN)

        self.publisher_front = self.create_publisher(Bool, 'ir_front', 10)
        self.publisher_back = self.create_publisher(Bool, 'ir_back', 10)
        self.timer = self.create_timer(0.5, self.publish_ir_data)

    def publish_ir_data(self):
        ir_state_front = not GPIO.input(self.ir_pin_front)
        ir_state_back = not GPIO.input(self.ir_pin_back)
        msg_front = Bool()
        msg_back = Bool()
        msg_front.data = bool(ir_state_front)
        msg_back.data = bool(ir_state_back)
        self.publisher_front.publish(msg_front)
        self.publisher_back.publish(msg_back)
        self.get_logger().info(f"Front IR: {msg_front.data}, Back IR: {msg_back.data}")

    def on_shutdown(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_node = IRSensorPublisher()
    try:
        rclpy.spin(ir_sensor_node)
    finally:
        ir_sensor_node.on_shutdown()
        ir_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

