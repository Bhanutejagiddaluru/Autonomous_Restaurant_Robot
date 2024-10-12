import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

###
## additional tool for local testing
## take screenshot from ros image
###

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
        Image, 
        '/camera/image_raw', 
        self.listener_callback, 
        10)
        self.subscription
        self.br = CvBridge()

        self.counter = 1

    def listener_callback(self, data):
        # self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)

        if cv2.waitKey(1) == ord('w'):
            cv2.imwrite("screenshot_images/" + str(self.counter) + ".jpg", current_frame) 
            self.counter += 1
            print ("write image " + str(self.counter))
            cwd = os.getcwd()
            print(cwd)

        cv2.imshow("camera", current_frame)



def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
