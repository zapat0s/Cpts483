#!/usr/bin/env python
#Nick Hayward
#Joshua Clark
#Lab 4

import rospy
import cv2
import numpy as np

#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

# main file -- if we "import move" in another file, this code will not execute.
if __name__== "__main__":
    # first thing, init a node!
    rospy.init_node('lab4')

    # publish to cmd_vel
    p = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)

    img_rgb = cv2.imread("orange.jpeg")
    
    #Makes Copy
    img_circle = img_rgb.copy() 
    
    #Makes a mask using the HSV values
    ORANGE_MIN = np.array([5, 100, 100],np.uint8)
    ORANGE_MAX = np.array([17, 255, 255],np.uint8)

    img_hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(img_hsv, ORANGE_MIN, ORANGE_MAX)


    #Finds HSV values
    '''
    orange = np.uint8([[[255,200,150]]])
    hsv_orange = cv2.cvtColor(orange,cv2.COLOR_RGB2HSV)
    print hsv_orange
    '''
    
    #Draw contour
    ret,thresh = cv2.threshold(mask,127,255,0)              #Creates threshold
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)      
    
    #Assigns cnt
    cnt = contours[0]
        
    #Draws contour lines
    cv2.drawContours(img_rgb,contours,-1,(0,255,0),3)
    
    #Fills contour
    #cv2.drawContours(img_rgb,contours,-1,(0,255,0),-1)
    
    #Find midpoint
   
    tar_contour = None
    tar_contour_length = 0;
    
    for c in contours:
        if len(c) > tar_contour_length:
            tar_contour = c
            tar_contour_length = len(c)      
        
    print tar_contour_length
    
    
    #Draw circle
    (x,y),radius = cv2.minEnclosingCircle(tar_contour)
    center = (int(x),int(y))
    radius = int(radius)
    cv2.circle(img_circle,center,radius,(0,255,0),2)
    
    #Draw center
    cv2.circle(img_circle,center, 5,(0,255,0),3)
    
    '''
    if moments['m00'] != 0.0:
       cx = moments['m10']/moments['m00']
       cy = moments['m01']/moments['m00']
       centroid = (cx,cy)
    else:
       centroid = "Region has zero area"
     
       
    im2 = mask.copy() 
    c = Contour(mask,cnt) 
    
    (cx,cy) = contours.centroid
    
    cv2.circle(im2,(int(cx),int(cy)),5,(0,255,0),-1)
    cv2.putText(im2,'green : centroid',(20,20), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))   
    '''
    '''
    (x,y),radius = cv2.minEnclosingCircle(cnt)
    center = (int(x),int(y))
    radius = int(radius)
    cv2.circle(mask,center,radius,(0,255,0),2)
    '''
    #Displays Images
    #cv2.imshow('hsv', img_hsv)
    #cv2.imshow('contours', contours)
    cv2.imshow('mask', mask)
    cv2.imshow('new', img_rgb)
    cv2.imshow('circle', img_circle)
    
    #Closes image on key input
    cv2.waitKey(0)
    cv2.destroyAllWindows()

