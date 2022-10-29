import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class RecordData(Node):
    """ The RecordData is a Python object that encompasses a ROS node 
        that can process images from the camera and save the image frame
        by frame or save the video. """

    def __init__(self, image_topic):
        """ Initialize the Record Data """
        super().__init__('ball_tracker')

        self.isVideo = True                         # if true, records video instead of image frames
        self.video_name = "testing1.avi"            # saved video's name

        self.cv_image = None                        # the latest image from the camera
        self.image_num = 0                          # image frame number
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        if self.isVideo:
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path = os.path.join(video_path, 'dataset', self.video_name)
            self.result = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'MJPG'), 10, (1024, 768))
        else:
            self.image_path = os.path.dirname(os.path.realpath(__file__))
            self.image_path = os.path.abspath(os.path.join(video_path, os.pardir))
        self.create_subscription(Image, image_topic, self.process_image, 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        if not self.cv_image is None:
            cv2.imshow('video_window', self.cv_image)
            if self.isVideo:
                self.result.write(self.cv_image)
            else:
                img_path = os.path.join(self.image_path, 'dataset', 'NoLens')
                cv2.imwrite(img_path + str(self.image_num) + '.jpg', self.cv_image)
                self.image_num += 1
            cv2.waitKey(5)

if __name__ == '__main__':
    node = RecordData("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = RecordData("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()