import matplotlib
import numpy as np
import pudb
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import cv2
from terra_camera.waypoint import WaypointPublisher
from topological_nav.reachability import model_factory
from topological_nav.reachability.planning import NavGraph, NavGraphSPTM


class CameraPublisher(Node):
    def __init__(self, camera_id=1, stream_video=False):
        super().__init__('camera_publisher')
        self.stream_video=stream_video
        self.cap = cv2.VideoCapture(camera_id)
        self.publisher_ = self.create_publisher(Image, 'camera', 100)
        timer_frequency = 1 / 30
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
        frame = cv2.resize(frame, dsize=(64,64), interpolation=cv2.INTER_CUBIC)
        frame = np.flipud(frame)
        frame = np.fliplr(frame)

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

class GoalPublisher(WaypointPublisher):
    def __init__(self):
        super().__init__('goal_publisher')
        self.counter = 0
        self.subscription = self.create_subscription(Image, 'camera', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Image, 'goal', 1)
        #Basically only plan once for now
        self.replan_every_n_frames = 99999999999999999
        self.topological_map = self.get_topological_map()
        self.set_final_goal()

    def get_topological_map(self):
        sparsify_thres = 0.99
        nav_graph = NavGraph.from_save_file(
             self.model["sparsifier"],
             self.model["motion_policy"],
             sparsify_thres,
             "./data/aesb/graph.pickle",
         )
        return nav_graph

    def get_re(self, ob, goal):
        follower = self.model["follower"]
        goal = self.cv2_to_model_im(goal)
        ob = self.cv2_to_model_im(ob)
        return follower.sparsifier.predict_reachability(ob, goal),
        
    def set_final_goal(self):
        def get_img(name):
            a = cv2.resize(
                cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB),
                dsize=(64, 64),
                interpolation=cv2.INTER_CUBIC)
            return a

        self.final_goal = []

        self.final_goal.append(get_img('./data/13000.png'))
        self.final_goal.append(get_img('./data/13010.png'))
        self.final_goal.append(get_img('./data/13020.png'))
        self.final_goal.append(get_img('./data/13030.png'))
        self.final_goal.append(get_img('./data/13040.png'))
        self.final_goal.append(get_img('./data/13050.png'))
        self.final_goal.append(get_img('./data/13060.png'))
        self.final_goal.append(get_img('./data/13070.png'))
        self.final_goal.append(get_img('./data/13080.png'))
        self.final_goal.append(get_img('./data/13090.png'))
        self.final_goal.append(get_img('./data/13100.png'))

    def update_local_goal(self):
        for i in range(len(self.path)):
            self.display_node(self.path[i], save_name='./data/out/frame' + str(i).zfill(4) + '.png')

    #Used for debugging, takes in node such as (0,13426) and displays corresponding image.
    def display_node(self, node_id, save_name=None):
        nodes = list(self.topological_map.graph.nodes.items())
        for node in nodes:
            if node[0] == node_id:
                if save_name is not None:
                    img = node[1]['ob_repr']
                    img = np.swapaxes(img, 0, 2)
                    img = np.swapaxes(img, 1, 0)
                    matplotlib.image.imsave(save_name, img)
                else:
                    self.show_img(node[1]['ob_repr'])
                    self.show_img(node[1]['dst_repr'])

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #Update planning path every few calls to this.
        if self.counter % self.replan_every_n_frames == 0:
            matplotlib.image.imsave('./data/out/frameStart.png', image)
            matplotlib.image.imsave('./data/out/frameEnd.png', self.final_goal[5])
           # self.path = [(0, 13426), (0, 13439), (0, 13446), (0, 13452), (0, 13455), (0, 13456), (0, 13457), (0, 13458), (0, 13459), (0, 13460), (0, 13461), (0, 13462), (0, 13463), (0, 13464), (0, 13465), (0, 13468), (0, 13475), (0, 13500)]
            self.path = self.find_path(image)
            self.update_local_goal()
        self.counter += 1

    def find_path(self, ob):
        ob = self.cv2_to_model_im(ob)
        goal = self.cv2_to_model_im(self.final_goal[5])
        dst_repr = self.model['sparsifier'].get_dst_repr_single(goal)

        pu.db
        path, log_likelihood, extra = self.topological_map.find_path(ob, dst_repr, edge_add_thres=0.3, allow_subgraph=True)
        if path is None:
            self.get_logger().warning("[camera.py:GoalPublisher] No path found!")
        return path



def main(args=None):
    rclpy.init(args=args)

    #Video stream doesnt work when ssh into machine and then run docker. TODO
    camera_publisher = CameraPublisher(camera_id=0, stream_video=False)
    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()

def goal_main(args=None):
    rclpy.init(args=args)

    #Video stream doesnt work when ssh into machine and then run docker. TODO
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
