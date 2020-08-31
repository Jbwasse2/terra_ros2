import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pudb
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, Twist, Vector3
from rclpy.node import Node
from rmp_nav.common.utils import (get_gibson_asset_dir, get_project_root,
                                  pprint_dict, str_to_dict)
from rmp_nav.simulation import agent_factory, sim_renderer
from sensor_msgs.msg import Image
from topological_nav.reachability import model_factory

os.environ["CUDA_VISIBLE_DEVICES"] = "0"

class WaypointPublisher(Node):

    def __init__(self, create_graphic=False):
        super().__init__('waypoint_publisher')
        self.create_graphic = create_graphic
        self.model = self.get_model()
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 100)
        self.publisher_ = self.create_publisher(Twist, 'terra_command_twist', 100)
        self.bridge = CvBridge()
        self.set_goal()
        self.get_logger().info('Created Waypoint Node')

    def get_model(self):
        model = model_factory.get("model_12env_v2_future_pair_proximity_z0228")(
            device="cuda"
        )
        return model

    def set_goal(self):
        self.goal = [
            cv2.resize(
                cv2.cvtColor(cv2.imread("./data/last_framebk.jpg"), cv2.COLOR_BGR2RGB),
                dsize=(64, 64),
                interpolation=cv2.INTER_CUBIC,
            )
            for i in range(11)
        ]
        self.goal_show = cv2.cvtColor(self.goal[0], cv2.COLOR_RGB2BGR)

    def create_waypoint_message(self, waypoint, reachability_estimator):
        lin = Vector3()
        angular = Vector3()
        #item converts from numpy float type to python float type
        lin.x = float(waypoint[0].item()) / 4
        lin.y = float(waypoint[1].item()) / 4
        #msg.z = reachability_estimator.item()
        lin.z = 0.0
        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0
        msg = Twist()
        msg.linear = lin
        msg.angular = angular
        return msg

    def image_callback(self, msg):
        self.get_logger().info('I heard {0}'.format(str(msg.header)))
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #The above converts the image to RGB
        waypoint, reachability_estimator = self.get_wp(image, self.goal)
        msg = self.create_waypoint_message(waypoint, reachability_estimator)
        self.publisher_.publish(msg)
        #The arrow is strictly for visualization, not used for navigation
        if self.create_graphic:
            arrow_start = (32,20)
            arrow_end = (32 + int(10 * -waypoint[1]), 20 + int(10 * -waypoint[0]))
            color = (0, 0, 255) #Red
            thickness = 1
            #cv2 like BGR because they like eating glue
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            image = np.hstack((image, self.goal_show))
            image = cv2.arrowedLine(image, arrow_start, arrow_end, color, thickness)
            (height, width, _) = image.shape
            image = cv2.resize(image, dsize=(width * 10, height * 10), interpolation=cv2.INTER_CUBIC)

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

    def show_img(self, img):
        img = np.swapaxes(img, 0, 2)
        img = np.swapaxes(img, 1, 0)
        plt.imshow(img)
        plt.show()

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

    waypoint_publisher = WaypointPublisher(create_graphic=True)

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
