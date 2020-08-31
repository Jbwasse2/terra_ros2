#the purpose of this file/package is to be able to debug the camera feed to make sure it is outputting results as expected
import numpy as np
import pudb
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import cv2


class DebugCamera(Node):
    def __init__(self):
        super().__init__('DebugCamera')
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 100)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('I heard {0}'.format(str(msg.header)))
        self.get_logger().info('This is an upscaled image, the original is 64x64')
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #The above converts the image to RGB, but we want it to stay BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image = cv2.resize(image, dsize=(500,500), interpolation=cv2.INTER_CUBIC)
        image = np.fliplr(image)
        #The arrow is strictly for visualization, not used for navigation
        cv2.imshow('frame', image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.__del__()

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = DebugCamera()

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()
