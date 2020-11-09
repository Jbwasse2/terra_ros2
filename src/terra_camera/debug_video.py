#the purpose of this file/package is to be able to debug the camera feed to make sure it is outputting results as expected
import numpy as np
import pudb

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class DebugCamera(Node):
    def __init__(self):
        super().__init__('DebugCamera')
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 100)
        self.bridge = CvBridge()
        self.counter = 0

    def image_callback(self, msg):
        self.get_logger().info('I heard {0}'.format(str(msg.header)))
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #The above converts the image to RGB, but we want it to stay BGR
#        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        #The arrow is strictly for visualization, not used for navigation
        counter_string = str(self.counter).rjust(5,'0')
        cv2.imwrite('./data/trajectory/' + counter_string + '.tiff', image)
        self.counter += 1 

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = DebugCamera()

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()
