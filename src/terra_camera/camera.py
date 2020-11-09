import matplotlib
import networkx as nx
import numpy as np
import pudb
from tqdm import tqdm

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
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
        return follower.sparsifier.predict_reachability(ob, goal),
        
    def set_final_goal(self):
        def get_img(name):
            a = cv2.resize(
                cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB),
                dsize=(64, 64),
                interpolation=cv2.INTER_CUBIC)
            return a

        self.final_goal = []

        self.final_goal.append(get_img('./data/frame05700.png'))
        self.final_goal.append(get_img('./data/frame05710.png'))
        self.final_goal.append(get_img('./data/frame05720.png'))
        self.final_goal.append(get_img('./data/frame05730.png'))
        self.final_goal.append(get_img('./data/frame05740.png'))
        self.final_goal.append(get_img('./data/frame05750.png'))
        self.final_goal.append(get_img('./data/frame05760.png'))
        self.final_goal.append(get_img('./data/frame05770.png'))
        self.final_goal.append(get_img('./data/frame05780.png'))
        self.final_goal.append(get_img('./data/frame05790.png'))
        self.final_goal.append(get_img('./data/frame05800.png'))

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
        #Update planning path every few calls to this.
        if self.counter % self.replan_every_n_frames == 0:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            matplotlib.image.imsave('./data/out/frameStart.png', image)
            matplotlib.image.imsave('./data/out/frameEnd.png', self.final_goal[5])
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.path = self.find_path(image)
            self.update_local_goal()
        self.counter += 1

    def get_path(self, ob, goal, keep_top=10):
        assert isinstance(goal, np.ndarray)
        assert isinstance(ob, np.ndarray)
        #find starting node
        nodes = list(self.topological_map.graph.nodes)
        img_lookup = dict(self.topological_map.graph.nodes.items())
        #keeps a list of (reachability estimator, node_label)
        re_tuples = []
        self.get_logger().info("[camera.py:GoalPublisher] Localizing in Graph...")
        for node in tqdm(nodes):
            node_img = img_lookup[node]['dst_repr']
            re = self.get_re(ob, node_img)
            re_tuples.append((re, node))
        re_tuples.sort(key=lambda x: x[0])
        self.get_logger().info("[camera.py:GoalPublisher] Best RE is " + str(re_tuples[-1]))
        #Find Goal Node
        best_re_goal = (0.92, (0,5775))
        self.get_logger().info("[camera.py:GoalPublisher] Finding Goal in Map...")
#        for node in tqdm(nodes):
#            node_img = img_lookup[node]['dst_repr']
#            re = self.get_re(goal, node_img)
#            if re > best_re_goal:
#                best_re_goal = (re, node)
        self.get_logger().info("[camera.py:GoalPublisher] Found goal in map, Reachability Estimator is " + str(best_re_goal[0]))
        for start in range(1,keep_top+1):
            try:
                path = nx.dijkstra_path(self.topological_map.graph, re_tuples[-start][1], best_re_goal[1])
            except Exception as e:
                path = None
            if path is not None:
                self.get_logger().info("[camera.py:GoalPublisher] Found self in graph, Reachability Estimator is " + str(re_tuples[-start][0]))
                a = path[0]
                b = path[-1]
                a_img = img_lookup[a]['ob_repr']
                b_img = img_lookup[b]['ob_repr']
                self.show_img(a_img)
                self.show_img(b_img)
                break
        return path

    def find_path(self, ob):
        ob = self.cv2_to_model_im(ob)
        goal = self.cv2_to_model_im(self.final_goal[5])
        dst_repr = self.model['sparsifier'].get_dst_repr_single(goal)
        path = self.get_path(ob, goal)
        #path, log_likelihood, extra = self.topological_map.find_path(ob, dst_repr, edge_add_thres=0.7, allow_subgraph=True)
        if path is None:
            self.get_logger().warning("[camera.py:GoalPublisher] No path found!")
        self.get_logger().debug("[camera.py:GoalPublisher] Log Likelihood: " + str(log_likelihood))
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
