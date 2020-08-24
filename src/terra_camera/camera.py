import numpy as np

import cv2
import rclpy
import pudb
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class CameraPublisher(Node):

    def __init__(self, camera_id=1, stream_video=False):
        super().__init__('camera_publisher')
        pu.db
        self.stream_video=stream_video
        self.cap = cv2.VideoCapture(camera_id)
        self.publisher_ = self.create_publisher(Image, 'camera', 1)
        timer_frequency = 1 / 60
        self.timer = self.create_timer(timer_frequency, self.timer_callback)
        self.counter = 0
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        msg = Image()
        header = Header()
        header.frame_id = str(self.counter)
        header.stamp = self.get_clock().now().to_msg()

        if not ret:
            self.get_logger().warning('Publishing RANDOM IMAGE "%s"' % str(header.stamp) )
            frame = np.random.randint(255, size=(480,640,3), dtype=np.uint8)
        if self.stream_video:
            cv2.imshow('frame', frame )
            cv2.waitKey(1)

        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        value = self.bridge.cv2_to_imgmsg(frame.astype(np.uint8))
        msg.data = value.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Image "%s"' % str(header.stamp) )
        self.counter += 1

    def __del__(self):
        # When everything done, release the capture
        self.cap.release()

def robot_main(args=None):
    rclpy.init(args=args)

    camera_publisher = RobotCameraPublisher()
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher(camera_id=1, stream_video=True)
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
