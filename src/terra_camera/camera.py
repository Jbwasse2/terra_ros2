import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pudb
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from terra_camera.waypoint import WaypointPublisher
from topological_nav.reachability import model_factory
from topological_nav.reachability.planning import NavGraph, NavGraphSPTM


matplotlib.use('TkAgg') 
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
        self.get_logger().info("Finished Building GoalPublisher")

    def get_topological_map(self):
        sparsify_thres = 0.99
        nav_graph = NavGraph.from_save_file(
             self.model["sparsifier"],
             self.model["motion_policy"],
             sparsify_thres,
             "./data/aesb/fresh.pickle",
         )
        return nav_graph

    def get_re(self, ob, goal):
        follower = self.model["follower"]
        goal = self.cv2_to_model_im(goal)
        ob = self.cv2_to_model_im(ob)
        return follower.sparsifier.predict_reachability(ob, goal),
        
    def set_final_goal(self):
        def get_img(name):
             #   cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB),
            a = cv2.resize(
                cv2.imread(name),
                dsize=(64, 64),
                interpolation=cv2.INTER_CUBIC)
            return a

        self.final_goal = []

        self.final_goal.append(get_img('./data/nodeDST_000_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_001_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_002_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_003_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_004_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_005_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_006_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_007_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_008_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_009_00549.png'))
        self.final_goal.append(get_img('./data/nodeDST_010_00549.png'))

    def save_trajectory(self):
        for i in range(len(self.path)):
            self.display_node(self.path[i], save_name='./data/out/frame' + str(i).zfill(4) + '.png')

    def get_images_in_graph(self):
        nodes = list(self.topological_map.graph.nodes.items())
        for counter, node in enumerate(nodes):
            img = node[1]['ob_repr']
            img = np.swapaxes(img, 0, 2)
            img = np.swapaxes(img, 1, 0)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            save_name = './data/out/nodeOB' + str(counter).zfill(5) + '.png'
            cv2.imwrite(save_name, img*255)
            imgdst = node[1]['dst_repr']
            for i in range(len(imgdst)):
                img = imgdst[i]
                img = np.swapaxes(img, 0, 2)
                img = np.swapaxes(img, 1, 0)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                save_name = './data/out/nodeDST_' +str(i).zfill(3) + '_' + str(counter).zfill(5) + '.png'
                cv2.imwrite(save_name, img*255)


    # takes in node such as (0,13426) and saves/returns corresponding image
    def display_node(self, node_id, save_name=None):
        nodes = list(self.topological_map.graph.nodes.items())
        for node in nodes:
            if node[0] == node_id:
                if save_name is not None:
                    img = node[1]['ob_repr']
                    img = np.swapaxes(img, 0, 2)
                    img = np.swapaxes(img, 1, 0)
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(save_name, img*255)
                else:
                    img = node[1]['ob_repr']
                    img = np.swapaxes(img, 0, 2)
                    img = np.swapaxes(img, 1, 0)
                    return img

    def send_trajectory_images(self):
        msg = Image()
        header = Header()
        header.frame_id = str(self.counter)
        header.stamp = self.get_clock().now().to_msg()

        images = []
        for path in self.path:
            images.append(self.display_node(path))
        frame = np.vstack(images)
        frame = frame * 255

        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        value = self.bridge.cv2_to_imgmsg(frame.astype(np.uint8))
        msg.data = value.data
        self.publisher_.publish(msg)
        self.get_logger().info('[Goal Publisher] Publishing Trajectory')

    def image_callback(self, msg):
        #Update planning path every few calls to this.
        if self.counter % self.replan_every_n_frames == 0:
            self.get_logger().info("[camera.py:GoalPublisher] Getting Trajectory Path")
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            #cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB),
#            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.path = self.find_path(image)
            self.send_trajectory_images()
            self.save_trajectory()
        self.counter += 1

    def find_path(self, ob):
        cv2.imwrite('./data/out/frameStart.png', ob)
        cv2.imwrite('./data/out/frameEnd.png', self.final_goal[5])
        ob = self.cv2_to_model_im(ob)
        goal = self.cv2_to_model_im(self.final_goal[5])
        dst_repr = self.model['sparsifier'].get_dst_repr_single(goal)
        path, log_likelihood, extra = self.topological_map.find_path(ob, dst_repr, edge_add_thres=0.7, allow_subgraph=True)
        if path is None:
            self.get_logger().warning("[camera.py:GoalPublisher] No path found!")
        if len(path) < 11:
            self.get_logger().warning("[camera.py:GoalPublisher] Short Path Found!")
        self.get_logger().info("[camera.py:GoalPublisher] Log Likelihood: " + str(log_likelihood))
        return path



def main(args=None):
    rclpy.init(args=args)

    #Video stream doesnt work when ssh into machine and then run docker. TODO
    camera_publisher = CameraPublisher(camera_id=1, stream_video=False)
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
