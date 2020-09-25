import matplotlib.pyplot as plt
import numpy as np
import pudb

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class VideoPublisher(Node):

    def __init__(self):
        super().__init__('video_publisher')
        self.get_logger().info('Getting Video Trajectory')
        self.trajectory = self.get_video_trajectory()
        self.get_logger().info('Finished Getting Video Trajectory')
        self.publisher_ = self.create_publisher(Image, 'camera', 1)
        timer_frequency = 1 / 24
        self.timer = self.create_timer(timer_frequency, self.timer_callback)
        self.counter = 0
        self.bridge = CvBridge()
        #For debugging images
        import matplotlib as mpl
        mpl.use('TkAgg')

    def timer_callback(self):
        video_length = self.trajectory.shape[0]
        assert video_length is not 0
        frame = self.trajectory[self.counter % video_length, :, :, :]
        msg = Image()
        header = Header()
        header.frame_id = str(self.counter)
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        value = self.bridge.cv2_to_imgmsg(frame.astype(np.uint8))
        msg.data = value.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Image "%s"' % str(header.stamp) )
        self.counter += 1

    def get_video_trajectory(self):
        cap = cv2.VideoCapture("./data/make_video/out.mp4")
        if cap.isOpened() == False:
            IOError("Error opening video stream or file")

        # Read until video is completed
        trajectory = []
        while cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()

            if ret == True:
                # Display the resulting frame
                frame = cv2.resize(frame, dsize=(64, 64), interpolation=cv2.INTER_CUBIC)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                trajectory.append(frame)
            else:
                break
        # When everything done, release the video capture object
        cap.release()
        # Closes all the frames
        return np.asarray(trajectory)

    def __del__(self):
        # When everything done, release the capture
        pass


def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
