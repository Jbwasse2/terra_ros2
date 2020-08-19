#This code will generalize around the tcp/ip comms that Mateus made. In order to get this to work you can either compile his branches, or you can ask Arun/Justin for the compiled binary.
#This code will subscribe to the topic "terra_command_x" and will do the appropiate actions based on the message. At the moment this is just vx,vy,vz,wx,wy,wz commands to the robot.
import socket
import time

import rclpy
from geometry_msgs.msg import Point32, Twist, Vector3
from rclpy.node import Node


#Twist, Vector3 linear, Vector3 angular
#Vector3 x,y,z float64
#Creates socket with terrasentia to communicate over.
#linear in m/s, angular in rad/s
class TerraComm(Node):
    def __init__(self, host="192.168.1.135", port=51717):
        super().__init__('terra_comm')
        #The controller should be at this IP
        self.host = host
        #The port that Mateus has defined the TCP/IP to comm over.
        self.port  = port
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
        s.connect((self.host, self.port))
        #TODO: some assertion to verify socket works
        return s

    #Creates message that gets sent to the terrasentia
    #This code is largely from Arun's code
    def create_terrasentia_message(self, linear, angular):
        header = "$CMD," 
        data_msg = "," + str(time.time()) + "," + str(linear.x) + "," + str(linear.y) + "," + str(linear.z) + "," + str(angular.x) + "," + str(angular.y) + "," + str(angular.z)
        bytes_count = len(header) + len(data_msg)
        message = header + str(bytes_count + len(str(bytes_count))) + data_msg
        assert isinstance(message, str)
        return message

#This dummy will test the message passsing to the TS.
#This will open a socket on a localhost port, and test that message passing works over socket.
class TerraCommDummyServer(Node):
    def __init__(self):
        super().__init__('terra_dummy_twist_server')
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.bind(('localhost', 51717))
        serversocket.listen(1) # become a server socket, maximum 1 connections
        self.get_logger().info('[TerraCommDummy] Create Dummy Server')
        while True:
            connection, address = serversocket.accept()
            buf = connection.recv(64)
            if len(buf) > 0:
                print(buf)
                break

class TerraCommDummyClient(TerraComm):
    def __init__(self):
        super().__init__(host='localhost')
        #Create server side socket
        linear = Vector3()
        angular = Vector3()

        linear.x = 1.0
        linear.y = 2.0
        linear.z = 3.0

        angular.x = 4.0
        angular.y = 5.0
        angular.z = 6.0

        terra_msg = self.create_terrasentia_message(linear, angular)
        self.get_logger().info('[TerraCommDummy] Create Dummy Client')
        while(1):
            self.socket.sendall(bytes(terra_msg, 'utf-8'))
            time.sleep(1)




def twist_dummy_server_main(args=None):
    rclpy.init(args=args)

    terraComm = TerraCommDummyServer()

    rclpy.spin(terraComm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    terraComm.destroy_node()
    rclpy.shutdown()

def twist_dummy_client_main(args=None):
    rclpy.init(args=args)

    terraComm = TerraCommDummyClient()

    rclpy.spin(terraComm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    terraComm.destroy_node()
    rclpy.shutdown()
def twist_main(args=None):
    rclpy.init(args=args)

    terraComm = TerraComm()

    rclpy.spin(terraComm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    terraComm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
