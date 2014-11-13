#!/usr/bin/env python
#Nick Hayward
#Joshua Clark
#Josh Woldstad
#Andrew Kim
#Lab 6

import rospy, cv2, math
import numpy as np

from threading import Lock

from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class ARImageTracker:
    # Converts from ROS image to OpenCV Image
    def ToOpenCV(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
            return cv_image
        except CvBridgeError, e:
            print e
            raise Exception("Failed to convert to OpenCV image")

    # Locates center of Orange Ball on image. Returns (-1,-1), 0 if ball is not found
    def BallLocation(self, cv_image):
        # Convert image to hsv
        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Makes a mask using the HSV values
        ORANGE_MIN = np.array([10, 120, 120],np.uint8)
        ORANGE_MAX = np.array([17, 255, 255],np.uint8)
        mask = cv2.inRange(img_hsv, ORANGE_MIN, ORANGE_MAX)

        #Draw contour
        ret,thresh = cv2.threshold(mask,127,255,0)              #Creates threshold
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        #Find midpoint
        tar_contour = None
        tar_contour_length = 0;
        if len(contours) == 0:
            return (-1,-1), 0    # Cound not find the ball
        for c in contours:
            if len(c) > tar_contour_length:
                tar_contour = c
                tar_contour_length = len(c)

        return cv2.minEnclosingCircle(tar_contour)

    # Sets image data
    # Only called by Subscriber Callback
    def ReceiveImage(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

    def __init__(self):
        # Setup Publishers and Subscibers
        self.takeoffPublisher = rospy.Publisher('/ardrone/takeoff', Empty)
        self.landPublisher = rospy.Publisher('/ardrone/land', Empty)
        self.twistPublisher = rospy.Publisher('cmd_vel', Twist)
        self.subVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage) # try /ardrone/image_raw

        # Setup Window
        self.window = cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)

        # Setup Camera
        self.fov = 46.0
        self.image = None
        self.imageLock = Lock()
        self.bridge = CvBridge()

        # Init Messages
        empty_msg = Empty()
        twist_msg = Twist()

        # Takeoff
        print "Takeoff"
        for i in range(0, 10):
            self.takeoffPublisher.publish(empty_msg)
            rospy.sleep(0.05) # 10*0.5 = 5 seconds

        # Control Loop
        while not rospy.is_shutdown():
            cv_img = None
            self.imageLock.acquire()
            try:
                if self.image is not None:
                    cv_img = np.asarray(self.ToOpenCV(self.image))
            finally:
                self.imageLock.release()
            if cv_img is not None:
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.z = 0.0

                (ball_x, ball_y), radius = self.BallLocation(cv_img)
                # Draw center and show image
                if ball_x != -1 and ball_y != -1:
                    cv2.circle(cv_img, (int(ball_x), int(ball_y)), 3,(0,255,0),2)
                cv2.imshow("Camera", cv_img)
                cv2.waitKey(1)

                # Rotate and find ball
                if ball_x == -1 and ball_y == -1:
                    twist_msg.angular.z = 5.0
                    self.twistPublisher.publish(twist_msg)
                    continue

                # Use radius of ball to determine distance
                if radius >= 16.0:
                    # Stopping
                    for i in range(0, 10):
                        self.twistPublisher.publish(twist_msg)
                        rospy.sleep(0.05) # 10*0.5 = 5 seconds
                    # Landing
                    self.landPublisher.publish(empty_msg)
                    continue;
                # Calculate direction vector 
                height, width, depth = cv_img.shape
                vertical_angle = self.fov / (height / 2) * ((height / 2) - ball_y)
                horizontal_angle = self.fov / (width / 2) * ((width / 2) - ball_x)

                # Create Twist Message
                twist_msg.linear.x = 0.08
                twist_msg.linear.y = 0
                twist_msg.linear.z = math.sin(math.radians(vertical_angle))
                twist_msg.angular.z = math.radians(horizontal_angle)
                self.twistPublisher.publish(twist_msg)
        cv2.destroyAllWindows()

# main file -- if we "import move" in another file, this code will not execute.
if __name__== "__main__":
    # first thing, init a node!
    rospy.init_node('lab5')

    node = ARImageTracker()

    
