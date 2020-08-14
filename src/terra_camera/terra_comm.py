#This code will generalize around the tcp/ip comms that Mateus made. In order to get this to work you can either compile his branches, or you can ask Arun/Justin for the compiled binary.
#This code will subscribe to the topic "terra_command_x" and will do the appropiate actions based on the message. At the moment this is just vx,vy,vz,wx,wy,wz commands to the robot.
import socket
import time

from geometry_msgs.msg import Point32, Twist, Vector3
from rclpy.node import Node


#Twist, Vector3 linear, Vector3 angular
#Vector3 x,y,z float64
#Creates socket with terrasentia to communicate over.
#linear in m/s, angular in rad/s
class TerraComm(Node):
    def __init__(self):
        super().__init__('terra_comm')
        self.socket = self.create_socket()
        self.subscription = self.create_subscription(Twist, 'terra_command_twist', self.twist_callback, 1)
    def twist_callback(self, msg):
        assert isinstance(msg, Twist)
        linear = msg.linear
        angular = msg.angular
        self.get_logger().info('[TerraComm: Twist] Linear: {0},{1},{2} Angular: {3},{4},{5}'.format(str(linear.x),str(linear.y), str(linear.z), str(angular.x), str(angular.y), str(angular.z)))
        terra_msg = self.create_terrasentia_message(linear, angular)
        self.socket.sendall(bytes(terra_msg, 'utf-8'))

    def create_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #The controller should be at this IP
        host = "192.168.1.135"
        #The port that Mateus has defined the TCP/IP to comm over.
        port  = 51717
        s.connect((host, port))
        #TODO: some assertion to verify socket works
        return s

    #Creates message that gets sent to the terrasentia
    #This code is largely from Arun's code
    def create_terrasentia_message(self, linear, angular):
        header = "$CMD," 
        data_msg = "," + str(time.time()) + "," + str(linear.x) + "," + str(linear.y) + "," + str(linear.z) + "," + str(angular.x) + "," + str(angular.y) + "," + str(angular.z) + ","
        bytes_count = len(header) + len(data_msg)
        message = header + str(bytes_count + len(str(bytes_count))) + data_msg
        assert isinstance(message, str)
        return message



def twist_main(args=None):
    rclpy.init(args=args)

    terraComm = TerraComm()

    rclpy.spin(terraComm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
