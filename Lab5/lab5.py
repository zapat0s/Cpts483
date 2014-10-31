#!/usr/bin/env python
#Nick Hayward
#Joshua Clark
#Josh Woldstad
#Andrew Kim
#Lab 5

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

    # Locates center of Orange Ball
    def BallLocation(self, cv_image):
        # Convert image to hsv
        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #Makes a mask using the HSV values
        ORANGE_MIN = np.array([5, 100, 100],np.uint8)
        ORANGE_MAX = np.array([17, 255, 255],np.uint8)
        mask = cv2.inRange(img_hsv, ORANGE_MIN, ORANGE_MAX)

        #Draw contour
        ret,thresh = cv2.threshold(mask,127,255,0)              #Creates threshold
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
        #Find midpoint
        tar_contour = None
        tar_contour_length = 0;
        for c in contours:
            if len(c) > tar_contour_length:
                tar_contour = c
                tar_contour_length = len(c)

        (x,y),radius = cv2.minEnclosingCircle(tar_contour)
        center = (int(x),int(y))
        return center;

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
        self.twistPublisher = rospy.Publisher('cmd_vel', Twist)
        self.subVideo = rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage) # try /ardrone/image_raw

        self.image = None
        self.imageLock = Lock()

        self.bridge = CvBridge()

        # Takeoff DOESNT WORK
        #empty_msg = Empty()
        #self.takeoffPublisher.publish(empty_msg)

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
                # Calculate angle 
                fov = 46.0
                ball_x = ball_y = 0
                try:
                    (ball_x, ball_y) = self.BallLocation(cv_img)
                except cv2.error, e:
                    # Rotate to Find Ball
                    twist_msg = Twist()
                    twist_msg.angular.z = 5.0
                    self.twistPublisher.publish(twist_msg)
                    print "searching"
                    continue
                height, width, depth = cv_img.shape
                vertical_dif = (height / 2) - ball_y
                horizontal_dif = (width / 2) - ball_x
                vertical_angle = fov / (height / 2) * vertical_dif
                horizontal_angle = fov / (width / 2) * horizontal_dif
                # Create Twist Message
                twist_msg = Twist()
                twist_msg.linear.x = 1.0
                twist_msg.linear.y = math.sin(math.radians(horizontal_angle))
                twist_msg.linear.z = math.sin(math.radians(vertical_angle))
                self.twistPublisher.publish(twist_msg)
                # Uncomment to Draw center and show image
                #cv2.circle(cv_img, (ball_x, ball_y), 3,(0,255,0),2)
                #cv2.imshow('camera', cv_img)
                #cv2.waitKey(0)
                #cv2.destroyAllWindows()
                
                

# main file -- if we "import move" in another file, this code will not execute.
if __name__== "__main__":
    # first thing, init a node!
    rospy.init_node('lab5')

    node = ARImageTracker()

    
