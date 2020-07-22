
import numpy as np
import pudb

import matplotlib.pyplot as plt
import cv2
import rclpy
from rmp_nav.simulation import agent_factory, sim_renderer
from rmp_nav.common.utils import (
    get_gibson_asset_dir,
    get_project_root,
    pprint_dict,
    str_to_dict,
)
from topological_nav.reachability import model_factory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "1"

class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.model = self.get_model()
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Point32, 'waypoint', 1)
        self.bridge = CvBridge()
        self.set_goal()

    def get_model(self):
        model = model_factory.get("model_12env_v2_future_pair_proximity_z0228")(
            device="cuda"
        )
        return model

    def set_goal(self):
        self.goal = [
            cv2.resize(
                cv2.cvtColor(cv2.imread("./data/last_frame.jpg"), cv2.COLOR_BGR2RGB),
                dsize=(64, 64),
                interpolation=cv2.INTER_CUBIC,
            )
            for i in range(11)
        ]
        self.goal_show = cv2.cvtColor(self.goal[0], cv2.COLOR_RGB2BGR)

    def create_waypoint_message(self, waypoint, reachability_estimator):
        msg = Point32()
        #item converts from numpy float type to python float type
        msg.x = waypoint[0].item()
        msg.y = waypoint[1].item()
        msg.z = reachability_estimator.item()
        return msg

    def image_callback(self, msg):
        self.get_logger().info('I heard {0}'.format(str(msg.header)))
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #The above converts the image to RGB, but we want it to stay BGR
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        waypoint, reachability_estimator = self.get_wp(image, self.goal)
        msg = self.create_waypoint_message(waypoint, reachability_estimator)
        self.publisher_.publish(msg)
        #The arrow is strictly for visualization, not used for navigation
        arrow_start = (32,20)
        arrow_end = (32 + int(10 * -waypoint[1]), 20 + int(10 * -waypoint[0]))
        color = (0, 0, 255) #Red
        thickness = 1
        image = np.hstack((image, self.goal_show))
        image = cv2.arrowedLine(image, arrow_start, arrow_end, color, thickness)
        cv2.imshow('frame', image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            self.__del__()

    def get_wp(self, ob, goal):
        agent = agent_factory.agents_dict[self.model["agent"]]()
        follower = self.model["follower"]
        goal = self.cv2_to_model_im(goal)
        ob = self.cv2_to_model_im(ob)
        return (
            follower.motion_policy.predict_waypoint(ob, goal),
            follower.sparsifier.predict_reachability(ob, goal),
        )

    #Cv2 gives images in BGR, and from 0-255
    #We want RGB and from 0-1
    #Can also get list/ np array of images, this should be handled
    def cv2_to_model_im(self,im):
        im = np.asarray(im)
        assert len(im.shape) == 3 or len(im.shape) == 4
        if len(im.shape) == 3:
            im = np.swapaxes(im, 0, 2)
            im = np.swapaxes(im, 1, 2)
            im = np.asarray(im)
            im = (im / 255).astype("float32")
        else:
            im = np.swapaxes(im, 1, 3)
            im = np.swapaxes(im, 2, 3)
            im = np.asarray(im)
            im = (im / 255).astype("float32")
        return im


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
