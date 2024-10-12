import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

###
## Additional tool for local testing
## Convert your camera to ROS topic
###

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # Ensure the topic name is correctly closed with a quote and bracket
        self.publisher_ = self.create_publisher(Image, '/real_camera/image', 10)
        timer_period = 0.1  # in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the video capture with the camera index
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the OpenCV image to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().error('Error capturing image')

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    except BaseException as e:
        print('Exception:', e)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

