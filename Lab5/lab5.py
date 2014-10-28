#!/usr/bin/env python
#Nick Hayward
#Joshua Clark
#Lab 4

import rospy
import cv2
import numpy as np

#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist


class ARImageTacker:
    def __init__(self):
        # Setup Publishers and Subscibers
        self.twistPublisher = rospy.Publisher('cmd_vel', Twist)
        self.subVideo = rospy.Subscriber('/ardrone/front/image_raw', Image, self.ReceiveImage)

        self.image = None
        self.imageLock = Lock();

        # Control Loop
        while True:
            cv_image = None
            self.imageLock.acquire()
            try:
                if self.image is not None:    
                    cv_img = ToOpenCV(self.image)
            finally:
                self.imageLock.release()
            if cv_image is not None:
                cv2.imshow('camera', cv_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                (x, y) = BallLocation(cv_img)
                if x - cv_image.width/2 < 10 and x - cv_image.width/2 >  -10: # Ball is centered hhorizontaly
                    if y - cv_image.height/2 < 10 and y - cv_image.height/2 > -10: # Ball is centered verticaly            
		        # fly forward
                        print "Flying forward"
                    else:
                        # change altitude
                        print "Changing altitude"
                else:
                    # rotate
                    print "Rotating"

    def ToOpenCV(ros_image):
        try:
            cv_image = bridge.imgmsg_to_cv(ros_image, "bgr8")
            return cv_image
        except CvBridgeError, e:
            print e
            raise Exception("Failed to convert to OpenCV image")

    def BallLocation(cv_image):
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
        print tar_contour_length

        (x,y),radius = cv2.minEnclosingCircle(tar_contour)
        center = (int(x),int(y))
        return center;

    def ReceiveImage(self, data):
        self.imageLock.acquire()
        try:
            self.image = data
        finally:
            self.imageLock.release()

# main file -- if we "import move" in another file, this code will not execute.
if __name__== "__main__":
    # first thing, init a node!
    rospy.init_node('lab5')

    
