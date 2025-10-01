import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult

class HsvConverter(Node):
    def __init__(self):
        super().__init__('hsv_converter')

        self.get_logger().info("Start hsv converter.")

        # Create subscriber

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            '/image_hsv',
            10) 
        
        # Declare parameters 파라미터 슬라이드바로 조절
        param_desc_saturation_scale = ParameterDescriptor(
            description='Saturation scale factor',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)],
        )
        param_desc_value_scale = ParameterDescriptor(
            description='Value scale factor',
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)],
        )

        self.declare_parameter('saturation_scale', 1.0, param_desc_saturation_scale)
        self.declare_parameter('value_scale', 1.0, param_desc_value_scale)

        saturation_scale = self.get_parameter('saturation_scale').value
        value_scale = self.get_parameter('value_scale').value

        self.get_logger().info("Initial saturation_scale : " + str(saturation_scale))
        self.get_logger().info("Initial value_scale : " + str(value_scale))

        #파라미터가 변경 되었을 때 콜백 함수 호출
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

    def parameter_callback(self, params):
        for param in params:
            msg = param.name + " is changed to " + str(param.value)
            self.get_logger().info(msg)

        return SetParametersResult(successful=True)
    
    def image_callback(self, msg):

        # Get params
        saturation_scale = self.get_parameter('saturation_scale').value
        value_scale = self.get_parameter('value_scale').value

        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        H, S, V = cv2.split(hsv_image)

        # Adjust saturation and value channels
        S = np.clip(S * saturation_scale, 0, 255)
        S = np.uint8(S)
        V = np.clip(V * value_scale, 0, 255)
        V = np.uint8(V)

        # Merge channels back
        hsv_image = cv2.merge((H, S, V))

        # Convert HSV back to BGR
        result_img = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
        hsv_msg = self.cv_bridge.cv2_to_imgmsg(result_img, encoding="bgr8")

        # Publish the result
        self.publisher.publish(hsv_msg)

def main():
    rclpy.init()
    node = HsvConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()