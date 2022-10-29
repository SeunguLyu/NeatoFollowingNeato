from pydoc import ispackage
import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import os

class NeatoTrackerColor(Node):
    """ The NeatoTrackerColor is a Python object that encompasses a ROS node 
    that controls Neato to follow another Neato based on color detection.
    The node will process images once user gives initial pixel to start with
    and calculates centroid of binary image created to find the angle between
    the target and the neato. Once the angle is found, message is pulbished
    to move the Neato so that it can track another Neato. """
    def __init__(self, image_topic):
        """ Initialize the neato tracker based on color """
        super().__init__('neato_tracker_color')

        self.isTesting = True                       # if true, test the code with recorded video
        self.video_name = "tracking1.avi"           # name of the recording used to test the code located in dataset folder

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.binary_image = None                    # binary image used for object centroid detection

        self.lidar = None                           # lidar sensor data to detect objects around neato
        self.should_move = 0                        # neato will start tracking if this value is 1

        self.boundary_range = 30                    # boundary range for r,g,b lower/upper bound when a pixel is clicked

        self.current_rgb = (0,0,0)                  # saving current/previous rgb values to avoid cv2.inRange() error
        self.previous_rgb = (0,0,0)

        self.center_x = 0                           # centroid of the detected object
        self.center_y = 0
        self.tendency_x = 0.0                       # tendency of detected object away from the robot

        self.drive_msg = Twist()                    # message to pulbish to control robot movement
        self.isProportional = False                 # if true, use proportional speed based on LIDAR data
        self.linSpeed = 0.2                         # the base speed of the robot

        # initialize boundaries
        self.red_lower_bound, self.green_lower_bound, self.blue_lower_bound = 0, 0, 0
        self.red_upper_bound, self.green_upper_bound, self.blue_upper_bound = 255, 255, 255

        # get image from Neato
        if (not self.isTesting):
            self.create_subscription(Image, image_topic, self.process_image, 10)
            self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # get image from dataset
        if (self.isTesting):
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path = os.path.join(video_path, 'dataset', self.video_name)
            self.cap = cv2.VideoCapture(video_path)
            if (self.cap.isOpened() == False): 
                print("Unable to read camera feed")
        
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_scan(self, msg):
        """ Process LIDAR scan data from Neato """
        self.lidar = msg

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def get_centroid(self):
        """ Calculates centroid from the binary image created with the r/g/b boundaries """
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
        self.tendency_x = self.center_x / self.binary_image.shape[1] - 0.5

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window', cv2.WINDOW_NORMAL)
        
        cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_bound, 255, self.set_red_lower_bound)
        cv2.createTrackbar('red upper bound', 'binary_window', self.red_upper_bound, 255, self.set_red_upper_bound)

        cv2.createTrackbar('green lower bound', 'binary_window', self.green_lower_bound, 255, self.set_green_lower_bound)
        cv2.createTrackbar('green upper bound', 'binary_window', self.green_upper_bound, 255, self.set_green_upper_bound)

        cv2.createTrackbar('blue lower bound', 'binary_window', self.blue_lower_bound, 255, self.set_blue_lower_bound)
        cv2.createTrackbar('blue upper bound', 'binary_window', self.blue_upper_bound, 255, self.set_blue_upper_bound)
        
        cv2.createTrackbar('shoud_move', 'binary_window', 0, 1, self.set_should_move)
        
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        while True:
            # if testing mode, get image from video feed
            if self.isTesting:
                ret, frame = self.cap.read()
                self.cv_image = frame
                if ret:
                    self.run_loop()
                    self.get_centroid()
                else:
                    break
            else:
                self.run_loop()
                self.get_centroid()
                self.drive()

            time.sleep(0.1)
    
    def set_should_move(self, val):
        """ A callback function to handle the OpenCV slider to select neato tracking status """
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
        """ Process mouse events so that you can get the color values
            associated with a particular pixel in the camera images
            and set color boundaries for the binary image """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.current_rgb = (self.cv_image[y,x,2], self.cv_image[y,x,1], self.cv_image[y,x,0])
            self.set_boundaries(self.current_rgb)

    def set_boundaries(self, rgb):
        """ Set lower and upper boundaries for binary image based on
        rgb value given and boundary range specified in the beginning """
        self.red_lower_bound = (rgb[0] - self.boundary_range) if (rgb[0] - self.boundary_range) > 0 else 0
        self.green_lower_bound = (rgb[1] - self.boundary_range) if (rgb[1] - self.boundary_range) > 0 else 0
        self.blue_lower_bound = (rgb[2] - self.boundary_range) if (rgb[2] - self.boundary_range) > 0 else 0

        self.red_upper_bound = (rgb[0] + self.boundary_range) if (rgb[0] + self.boundary_range) < 255 else 255
        self.green_upper_bound = (rgb[1] + self.boundary_range) if (rgb[1] + self.boundary_range) < 255 else 255
        self.blue_upper_bound = (rgb[2] + self.boundary_range) if (rgb[2] + self.boundary_range) < 255 else 255

        cv2.setTrackbarPos('blue lower bound', 'binary_window', self.blue_lower_bound)
        cv2.setTrackbarPos('red lower bound', 'binary_window', self.red_lower_bound)
        cv2.setTrackbarPos('green lower bound', 'binary_window', self.green_lower_bound)

        cv2.setTrackbarPos('blue upper bound', 'binary_window', self.blue_upper_bound)
        cv2.setTrackbarPos('red upper bound', 'binary_window', self.red_upper_bound)
        cv2.setTrackbarPos('green upper bound', 'binary_window', self.green_upper_bound)

    def drive(self):
        """ Publish message to move the Neato based on the calculated 
        centroid of the binary image. By default, angular velocity is 
        proportional, but linear velocity is proportional only if 
        self.isProportional value is true. """
        if self.should_move == 1:
            self.drive_msg.angular.z = -self.tendency_x * 2
            if self.isProportional:
                if self.lidar != None:
                    angle = int(-self.tendency_x * 100)
                    distance = self.lidar.ranges[angle]
                    # ignore occasional 0 value for the distance
                    if distance < 0.5 and not distance == 0.0:
                        self.drive_msg.linear.x = 0.1
                    elif not distance == 0.0:
                        self.drive_msg.linear.x = (distance-0.5) * 0.2 + 0.1
                    else:
                        pass
            else:
                self.drive_msg.linear.x = float(self.linSpeed)           
        else:
            self.drive_msg.linear.x = 0.0
            self.drive_msg.angular.z = 0.0

        self.pub.publish(self.drive_msg)

    def run_loop(self): 
        """ Loop that runs to get a new binary image based on boundaries.
        Due to an error that occurs on certain rgb values, if it fails to 
        get a new binary image, resets the color boundaries back to the 
        last value that worked. Also visualizes centroid position on the 
        video window. """
        if not self.cv_image is None:
            lowerb = (self.blue_lower_bound, self.green_lower_bound, self.red_lower_bound)
            upperb = (self.blue_upper_bound, self.green_upper_bound, self.red_upper_bound)
            try:
                self.binary_image = cv2.inRange(self.cv_image, lowerb, upperb)
            except:
                print("Range Error Occured")
                self.set_boundaries((0,0,0))
                self.set_boundaries(self.previous_rgb)
            else:
                self.previous_rgb = self.current_rgb
                cv2.circle(self.cv_image, (self.center_x, self.center_y), 10, (128, 255, 0), -1)
                
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
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