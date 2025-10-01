import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)


        self.declare_parameter('camera_device', 0)
        self.camera_device = self.get_parameter('camera_device').value
        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('height', 480)
        self.height = self.get_parameter('height').value
        self.declare_parameter('frame_rate', 400)
        self.frame_rate = self.get_parameter('frame_rate').value

        self.cap = cv2.VideoCapture(self.camera_device)
        self.cv_bridge = CvBridge()
        time_period = 1.0 / self.frame_rate  # Convert frame rate to time period
        self.timer = self.create_timer(time_period, self.time_callback)  # Publish at 10 Hz

        self.get_logger().info("Camera Device : " + str(self.camera_device))
        self.get_logger().info("Video Width : " + str(self.width))
        self.get_logger().info("Video Height : " + str(self.height))
        self.get_logger().info("Frame Rate : " + str(self.frame_rate))

    def time_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (self.width, self.height))
            img = self.cv_bridge.cv2_to_imgmsg(frame,"bgr8")
            self.publisher.publish(img)
        else:
            self.get_logger().warn("Failed to capture frame from camera")

def main():
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()