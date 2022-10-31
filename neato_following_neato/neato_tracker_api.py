import os
import cv2
import time
import rclpy
import numpy as np
from pydoc import ispackage
from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NeatoTrackerAPI(Node):
    def __init__(self, image_topic):
        """ Initialize the neato tracker based on tracking API """
        super().__init__('neato_tracker_api')

        # 트랙커 객체 생성자 함수 리스트 ---①
        self.trackers = [cv2.legacy.TrackerBoosting_create,
                         cv2.TrackerMIL_create,
                         cv2.TrackerKCF_create,
                         cv2.legacy.TrackerTLD_create,
                         cv2.legacy.TrackerMedianFlow_create,
                         cv2.TrackerGOTURN_create, #버그로 오류 발생
                         cv2.TrackerCSRT_create,
                         cv2.legacy.TrackerMOSSE_create]
        self.trackerIdx = 0  # 트랙커 생성자 함수 선택 인덱스
        self.tracker = None
        self.isFirst = True
        self.isInit = None

        self.isTesting = True                       # if true, test the code with recorded video
        self.video_name = "tracking1.avi"           # name of the recording used to test the code located in dataset folder

        self.isRecording = False
        self.recording_name = "recording_centroid.avi"
        self.recording_name_binary = "recording_binary.avi"

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.binary_image = None                    # binary image used for object centroid detection

        self.lidar = None                           # lidar sensor data to detect objects around neato
        self.should_move = 0                        # neato will start tracking if this value is 1

        self.boundary_range = 30                    # boundary range for r,g,b lower/upper bound when a pixel is clicked

        # self.current_rgb = (0,0,0)                  # saving current/previous rgb values to avoid cv2.inRange() error
        # self.previous_rgb = (0,0,0)

        self.center_x = 0                           # centroid of the detected object
        self.center_y = 0
        self.tendency_x = 0.0                       # tendency of detected object away from the robot

        self.drive_msg = Twist()                    # message to pulbish to control robot movement
        self.isProportional = False                 # if true, use proportional speed based on LIDAR data
        self.linSpeed = 0.2                         # the base speed of the robot

        # # initialize boundaries
        # self.red_lower_bound, self.green_lower_bound, self.blue_lower_bound = 0, 0, 0
        # self.red_upper_bound, self.green_upper_bound, self.blue_upper_bound = 255, 255, 255

        # get image from dataset
        if (self.isTesting):
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path = os.path.join(video_path, 'dataset', self.video_name)
            self.cap = cv2.VideoCapture(video_path)
            if (self.cap.isOpened() == False): 
                print("Unable to read camera feed")

        # get image from Neato
        else:
            self.create_subscription(Image, image_topic, self.process_image, 10)
            self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # record video
        if (self.isRecording):
            video_path = os.path.dirname(os.path.realpath(__file__))
            video_path = os.path.abspath(os.path.join(video_path, os.pardir))
            video_path1 = os.path.join(video_path, 'dataset', self.recording_name)
            video_path2 = os.path.join(video_path, 'dataset', self.recording_name_binary)
            self.result1 = cv2.VideoWriter(video_path1, cv2.VideoWriter_fourcc(*'MJPG'), 10, (1024, 768))
            self.result2 = cv2.VideoWriter(video_path2, cv2.VideoWriter_fourcc(*'MJPG'), 10, (1024, 768))
        
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_scan(self, msg):
        """ Process LIDAR scan data from Neato """
        self.lidar = msg

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
                self.drive()

            time.sleep(0.1)

    def set_should_move(self, val):
        """ A callback function to handle the OpenCV slider to select neato tracking status """
        self.should_move = val

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
        win_name = "tracking API"
        if not self.cv_image is None:
            img_draw = self.cv_image.copy()
            if self.tracker is None: # 트랙커 생성 안된 경우
                cv2.putText(img_draw, "Press the Space to set ROI!!", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
            else:
                ok, bbox = self.tracker.update(self.cv_image)   # 새로운 프레임에서 추적 위치 찾기 ---③
                (x,y,w,h) = bbox
                if ok: # 추적 성공
                    cv2.rectangle(img_draw, (int(x), int(y)), (int(x + w), int(y + h)), (0,255,0), 2, 1)
                    cv2.circle(img_draw, (int(x + w/2), int(y + h/2)), radius=0, color=(255, 0, 0), thickness=2)
                    print("x: {}, y: {}".format((x + w/2), (y + h/2)))

                    self.center_x = int(x + w/2)
                    self.center_y = int(y + h/2)
                    self.tendency_x = self.center_x / self.cv_image.shape[1] - 0.5

                else : # 추적 실패
                    cv2.putText(img_draw, "Tracking fail.", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)

            trackerName = self.tracker.__class__.__name__
            cv2.putText(img_draw, str(trackerIdx) + ":"+trackerName , (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0),2,cv2.LINE_AA)

            cv2.imshow(win_name, img_draw)
            key = cv2.waitKey(10) & 0xff
            # 스페이스 바 또는 비디오 파일 최초 실행 ---④
            if key == ord(' ') or (self.isTesting and self.isFirst): 
                isFirst = False
                roi = cv2.selectROI(win_name, self.cv_image, False)  # 초기 객체 위치 설정
                if roi[2] and roi[3]:         # 위치 설정 값 있는 경우
                    self.tracker = self.trackers[self.trackerIdx]()    #트랙커 객체 생성 ---⑤
                    self.isInit = self.tracker.init(self.cv_image, roi)

            elif key in range(48, 56): # 0~7 숫자 입력   ---⑥
                trackerIdx = key-48     # 선택한 숫자로 트랙커 인덱스 수정
                if bbox is not None:
                    self.tracker = self.trackers[self.trackerIdx]() # 선택한 숫자의 트랙커 객체 생성 ---⑦
                    self.isInit = self.tracker.init(self.cv_image, bbox) # 이전 추적 위치로 추적 위치 초기화

            elif key == 27 : 
                return

            if (self.isRecording):
                self.result1.write(self.cv_image)
                frame = cv2.cvtColor(self.binary_image, cv2.COLOR_GRAY2RGB)
                self.result2.write(frame)
                
            cv2.imshow('video_window', self.cv_image)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = NeatoTrackerAPI("/camera/image_raw")
    node.run()

def main(args=None):
    rclpy.init()
    n = NeatoTrackerAPI("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


'''
# 트랙커 객체 생성자 함수 리스트 ---①
trackers = [cv2.legacy.TrackerBoosting_create,
            cv2.TrackerMIL_create,
            cv2.TrackerKCF_create,
            cv2.legacy.TrackerTLD_create,
            cv2.legacy.TrackerMedianFlow_create,
            cv2.TrackerGOTURN_create, #버그로 오류 발생
            cv2.TrackerCSRT_create,
            cv2.legacy.TrackerMOSSE_create]
trackerIdx = 0  # 트랙커 생성자 함수 선택 인덱스
tracker = None
isFirst = True

video_src = 0 # 비디오 파일과 카메라 선택 ---②
# video_src = "../dataset/try2.avi"
cap = cv2.VideoCapture(video_src)
fps = cap.get(cv2.CAP_PROP_FPS) # 프레임 수 구하기
delay = int(1000/fps)
win_name = 'Tracking APIs'
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print('Cannot read video file')
        break
    img_draw = frame.copy()
    if tracker is None: # 트랙커 생성 안된 경우
        cv2.putText(img_draw, "Press the Space to set ROI!!", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)
    else:
        ok, bbox = tracker.update(frame)   # 새로운 프레임에서 추적 위치 찾기 ---③
        (x,y,w,h) = bbox
        if ok: # 추적 성공
            cv2.rectangle(img_draw, (int(x), int(y)), (int(x + w), int(y + h)), (0,255,0), 2, 1)
            cv2.circle(img_draw, (int(x + w/2), int(y + h/2)), radius=0, color=(255, 0, 0), thickness=2)
            print("x: {}, y: {}".format((x + w/2), (y + h/2)))

        else : # 추적 실패
            cv2.putText(img_draw, "Tracking fail.", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2,cv2.LINE_AA)

    trackerName = tracker.__class__.__name__
    cv2.putText(img_draw, str(trackerIdx) + ":"+trackerName , (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0),2,cv2.LINE_AA)

    cv2.imshow(win_name, img_draw)
    key = cv2.waitKey(delay) & 0xff
    # 스페이스 바 또는 비디오 파일 최초 실행 ---④
    if key == ord(' ') or (video_src != 0 and isFirst): 
        isFirst = False
        roi = cv2.selectROI(win_name, frame, False)  # 초기 객체 위치 설정
        if roi[2] and roi[3]:         # 위치 설정 값 있는 경우
            tracker = trackers[trackerIdx]()    #트랙커 객체 생성 ---⑤
            isInit = tracker.init(frame, roi)

    elif key in range(48, 56): # 0~7 숫자 입력   ---⑥
        trackerIdx = key-48     # 선택한 숫자로 트랙커 인덱스 수정
        if bbox is not None:
            tracker = trackers[trackerIdx]() # 선택한 숫자의 트랙커 객체 생성 ---⑦
            isInit = tracker.init(frame, bbox) # 이전 추적 위치로 추적 위치 초기화

    elif key == 27 : 
        break
else:
    print( "Could not open video")
cap.release()
cv2.destroyAllWindows()
'''