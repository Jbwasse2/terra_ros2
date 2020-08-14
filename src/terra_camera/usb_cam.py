
import logging as log
import math
import os
import socket
import sys
import time
from argparse import SUPPRESS, ArgumentParser

import numpy as np

import cv2
#!/usr/bin/env python3
import roslaunch
import rospy
from openvino.inference_engine import IECore
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64


def build_argparser():
    parser = ArgumentParser(add_help=False)  
    args = parser.add_argument_group('Options')
    args.add_argument("-m", "--model", help="Required. Path to an .xml file with a trained model.", required=True,
                      type=str)
    return parser  

VERBOSE=True 

class smoothcurve:
    
    def __init__(self):
        #input heading in degrees
        
        self.b = 3.8
        self.c = 0.55
        self.d = 0.7
        self.yaw_deadband = 0.02
        self.prev_x = -1
        self.prev_heading = -1
    
    def update_ref_heading(self, distance_ratio, heading):
        self.heading = heading
        self.distance_ratio = distance_ratio
        self.lane_width = 0.735
        self.dl = self.distance_ratio*self.lane_width
        self.dr = self.lane_width-self.dl 

        pos_ref =0
        x_pos = -(pos_ref-(self.dr-self.dl))
        if self.prev_x ==-1 :
            filtered_x  = x_pos
        
        else:
            filtered_x = x_pos+0.96*(self.prev_x-x_pos)

        target_heading_angle = (-math.atan(self.c*(filtered_x/self.d)*math.pow(self.b,self.c)*math.pow((1/abs(filtered_x/self.d)),self.c+2)))
        
        if target_heading_angle < 0:
            target_heading_angle+=math.pi
        
        actual_heading_angle = math.fmod((90-self.heading),360.0)
        if actual_heading_angle<0:
            actual_heading_angle+=360

        heading_rad = heading*math.pi/180

        if self.prev_heading == -1:
            filtered_heading = actual_heading_angle
        else:
            filtered_heading = actual_heading_angle+0.96*(self.prev_heading-actual_heading_angle)
        
        self.prev_x=filtered_x
        self.prev_heading = filtered_heading
        
        return target_heading_angle*180/math.pi, filtered_heading


class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.output>3:
                self.output=3
            

            if self.output<-3:
                self.output=-3

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time       


class vision_controller:
   

    def __init__(self,loaded_net,input_blob,out_blob, pid_output, sock, p_field):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        
        self.exec_net = loaded_net
        self.input_blob = input_blob
        self.out_blob  = out_blob
        self.output_heading = rospy.Publisher('output/heading', Float64,queue_size = 1)
        self.output_distance = rospy.Publisher('output/distance_ratio', Float64,queue_size = 1)
        self.turn_rate = rospy.Publisher('output/turn_rate', Float64,queue_size = 1)
        self.target_heading_angle =rospy.Publisher('output/target_heading', Float64,queue_size = 1)
        self.filtered_heading =rospy.Publisher('output/filtered_heading', Float64,queue_size = 1)
        self.flip_image = rospy.Publisher('output/image_raw/compressed', CompressedImage,queue_size = 1)
        self.pid = pid_output
        self.sock = sock
        self.p_field = p_field
        self.latency = []
        # self.bridge = CvBridge()
         
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1, buff_size=2**24)
        if VERBOSE :
            print("subscribed to /usb_cam/image_raw/compressed")

   

    def callback(self, ros_data):
        start = rospy.get_time()
        '''Callback function of subscribed topic.
        Here images get converted and deep learning inference done'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)
       
         #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  
        flip_image = cv2.flip(image,0)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg',flip_image)[1]).tostring()


        image = cv2.cvtColor(flip_image, cv2.COLOR_BGR2RGB)
        
        h,w = image.shape[:2]
        ch, cw = int(0.2*h), int(0.2*w)
        image = image[ch:h-ch, cw:w-cw]
        image_dims = (640,360)        
        data = cv2.resize(image,image_dims)
        data = data/255
        data[:,:,0] = (data[:,:,0]-0.485)/0.229
        data[:,:,1] = (data[:,:,1]-0.456)/0.224
        data[:,:,2] = (data[:,:,2]-0.406)/0.225
        data = np.expand_dims(data,axis=0)
        data = np.transpose(data, axes=[0,3,1,2])
        
        # Start sync inference
        log.info("Starting inference in synchronous mode")
        res = self.exec_net.infer(inputs={self.input_blob: data})

        # Processing output blob
        log.info("Processing output blob")
        res = res[self.out_blob].flatten()
        print('Heading is {} degrees'.format(res[0]))
        print('Distance ratio is {}'.format(res[1]))

        heading = res[0] #heading in degrees
        distance_ratio = res[1] #represents dl/(dl+dr)  
        target_heading_angle, filtered_heading = self.p_field.update_ref_heading(distance_ratio,heading)
        self.pid.update(target_heading_angle-filtered_heading)
        #turn_rate = self.pid.output 
        turn_rate = -0.2*heading
        print('Turn rate is {}'.format(turn_rate))

        
        
        print("Sending to pi")
        header = "$CMD,"
        data_msg = ","+str(time.time())+",0.5,0,0,0,0," + str(turn_rate)
        bytes_count = len(header)+len(data_msg)
        sendData = header + str(bytes_count+len(str(bytes_count))) + data_msg

        print(sendData)
        print(len(sendData))
        self.sock.sendall(bytes(sendData, 'utf-8'))    
           
        
        #### Publish numpy topic  ####
        self.output_heading.publish(res[0])
        self.output_distance.publish(res[1])
        self.turn_rate.publish(turn_rate)
        self.target_heading_angle.publish(target_heading_angle)
        self.filtered_heading.publish(filtered_heading)
        self.flip_image.publish(msg)
        end = rospy.get_time()
        self.latency.append((end-start).to_sec())

        print("Average latency is {} seconds".format(sum(self.latency)/len(self.latency)))
    

def main():
    args = build_argparser().parse_args()
    rospy.init_node('vision_controller', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/companion/Downloads/usb_cam-test.launch"])
    launch.start()
    rospy.loginfo("started")
    '''Initializes and cleanup ros node'''
    log.basicConfig(format="[ %(levelname)s ] %(message)s", level=log.INFO, stream=sys.stdout)
    model_xml = args.model
    model_bin = os.path.splitext(model_xml)[0] + ".bin"

    # Plugin initialization for specified device and load extensions library if specified
    log.info("Creating Inference Engine")
    ie = IECore()
   
    # Read IR
    log.info("Loading network files:\n\t{}\n\t{}".format(model_xml, model_bin))
    net = ie.read_network(model=model_xml, weights=model_bin)      

    log.info("Preparing input blobs")
    input_blob = next(iter(net.inputs))
    out_blob = next(iter(net.outputs))        

    # Loading model to the plugin
    log.info("Loading model to the plugin")
    exec_net = ie.load_network(network=net, device_name="MYRIAD")

    #PID 
    pid_output = PID(P = 5,  I = 0, D= 0.2)
    pid_output.SetPoint = 0
    p_field = smoothcurve()

    #tcp socket programming
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = "192.168.1.135"
    port  = 51717
    s.connect((host, port))
    
    print("Socket Up and running")  


    ic = vision_controller(exec_net,input_blob, out_blob, pid_output,s, p_field)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image neural network prediction module")
    
 

if __name__ == '__main__':
    sys.exit(main() or 0)
feedback
