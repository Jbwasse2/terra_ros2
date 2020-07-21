
import numpy as np
import pudb

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.model = self.get_model()
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 1)
        self.bridge = CvBridge()

    def get_model(self):
        return None

    def image_callback(self, msg):
        self.get_logger().info('I heard {0}'.format(str(msg.header)))
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        pu.db
        cv2.imshow('frame', image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.__del__()

    def __del__(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
