import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import json
from os import path

class NeatoTrackerColor(Node):
    """ 
    """
    def __init__(self, image_topic):
        """ Initialize the neato tracker """
        super().__init__('neato_tracker_color')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.binary_image = None
        self.isTesting = True
        self.lidar = None

        self.boundary_range = 20
        self.red_lower_bound, self.green_lower_bound, self.blue_lower_bound = 181, 89, 95
        self.red_upper_bound, self.green_upper_bound, self.blue_upper_bound = 255, 141, 178

        # get image from Neato
        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

        # get image from dataset
        if (self.isTesting):
            self.cap = cv2.VideoCapture("/home/seungu/ros2_ws/src/NeatoFollowingNeato/dataset/try1.avi")
            if (self.cap.isOpened() == False): 
                print("Unable to read camera feed")

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.should_move = 0
        self.drive_msg = Twist()
        self.linSpeed = 0.1
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_scan(self, msg):
        self.lidar = msg

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def get_centroid(self):
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        self.center_x = self.center_x / self.binary_image.shape[1] - 0.5
        print('self center x normalized', self.center_x)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window', cv2.WINDOW_NORMAL)
        cv2.namedWindow('image_info')
        
        cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_bound, 255, self.set_red_lower_bound)
        cv2.createTrackbar('red upper bound', 'binary_window', self.red_upper_bound, 255, self.set_red_upper_bound)

        cv2.createTrackbar('green lower bound', 'binary_window', self.green_lower_bound, 255, self.set_green_lower_bound)
        cv2.createTrackbar('green upper bound', 'binary_window', self.green_upper_bound, 255, self.set_green_upper_bound)

        cv2.createTrackbar('blue lower bound', 'binary_window', self.blue_lower_bound, 255, self.set_blue_lower_bound)
        cv2.createTrackbar('blue upper bound', 'binary_window', self.blue_upper_bound, 255, self.set_blue_upper_bound)
        
        cv2.createTrackbar('shoud_move', 'binary_window', 0, 1, self.set_should_move)
        
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        while True:
            if self.isTesting:
                ret, frame = self.cap.read()
                self.cv_image = frame
            self.run_loop()
            self.drive()
            time.sleep(0.1)
    
    def set_should_move(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.should_move = val

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def set_red_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red upper bound """
        self.red_upper_bound = val

    def set_green_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.green_lower_bound = val

    def set_green_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red upper bound """
        self.green_upper_bound = val

    def set_blue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue lower bound """
        self.blue_lower_bound = val
        
    def set_blue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue upper bound """
        self.blue_upper_bound = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((200,500,3))
        if event == cv2.EVENT_LBUTTONDOWN:
            self.red_lower_bound = (self.cv_image[y,x,2] - self.boundary_range) if (self.cv_image[y,x,2] - self.boundary_range) > 0 else 0
            self.green_lower_bound = (self.cv_image[y,x,1] - self.boundary_range) if (self.cv_image[y,x,1] - self.boundary_range) > 0 else 0
            self.blue_lower_bound = (self.cv_image[y,x,0] - self.boundary_range) if (self.cv_image[y,x,0] - self.boundary_range) > 0 else 0

            self.red_upper_bound = (self.cv_image[y,x,2] + self.boundary_range) if (self.cv_image[y,x,2] + self.boundary_range) < 255 else 255
            self.green_upper_bound = (self.cv_image[y,x,1] + self.boundary_range) if (self.cv_image[y,x,1] + self.boundary_range) < 255 else 255
            self.blue_upper_bound = (self.cv_image[y,x,0] + self.boundary_range) if (self.cv_image[y,x,0] + self.boundary_range) < 255 else 255

            cv2.setTrackbarPos('blue lower bound', 'binary_window', self.blue_lower_bound)
            cv2.setTrackbarPos('red lower bound', 'binary_window', self.red_lower_bound)
            cv2.setTrackbarPos('green lower bound', 'binary_window', self.green_lower_bound)

            cv2.setTrackbarPos('blue upper bound', 'binary_window', self.blue_upper_bound)
            cv2.setTrackbarPos('red upper bound', 'binary_window', self.red_upper_bound)
            cv2.setTrackbarPos('green upper bound', 'binary_window', self.green_upper_bound)

        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SCRIPT_COMPLEX,
                    1,
                    (0,0,0))

    def drive(self):
        if self.should_move == 1:
            angle = int(-self.center_x)
            if self.lidar != None:
                distance = self.lidar.ranges[angle]
                self.drive_msg.angular.z = -self.center_x * 2
                if (distance < 0.3):
                    self.drive_msg.linear.x = float(self.linSpeed)
                else:
                    self.drive_msg.linear.x = (distance-0.3) * 0.5 + self.linSpeed
        else:
            self.drive_msg.linear.x = 0.0
            self.drive_msg.angular.z = 0.0

        self.pub.publish(self.drive_msg)

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound, self.green_lower_bound, self.red_lower_bound), (self.blue_upper_bound, self.green_upper_bound, self.red_upper_bound))
            # print(self.cv_image.shape)
            self.get_centroid()
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = NeatoTrackerColor("/camera/image_raw")
    node.run()

def main(args=None):
    rclpy.init()
    n = NeatoTrackerColor("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()