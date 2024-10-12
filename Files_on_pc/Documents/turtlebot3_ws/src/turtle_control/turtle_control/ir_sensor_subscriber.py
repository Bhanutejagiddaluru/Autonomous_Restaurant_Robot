import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class IRSensorSubscriber(Node):
    def __init__(self):
        super().__init__('ir_sensor_subscriber')
        self.subscription_front = self.create_subscription(
            Bool, 'ir_front', self.ir_front_callback, 10)
        self.subscription_back = self.create_subscription(
            Bool, 'ir_back', self.ir_back_callback, 10)

    def ir_front_callback(self, msg):
        self.get_logger().info(f'Received front IR sensor data: {"Object detected" if msg.data else "No object detected"}')

    def ir_back_callback(self, msg):
        self.get_logger().info(f'Received back IR sensor data: {"Object detected" if msg.data else "No object detected"}')

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_subscriber = IRSensorSubscriber()
    rclpy.spin(ir_sensor_subscriber)
    ir_sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

