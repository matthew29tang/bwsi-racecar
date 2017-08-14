#!/usr/bin/python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

class echoNode:
    def __init__(self):

        ###########################################
        # TUNABLE_THRESH_GREEN = [lowerhsv, upperhsv]
        self.TUNABLE_THRESH_GREEN = [np.array([1, 150, 120]), np.array([20,255,255])]

        #TUNABLE_CONTOUR_AREA = minimumarea
        self.TUNABLE_CONTOUR_AREA = 999999 #edit this
        ###########################################

        rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.proc_img)
        
        self.vid_cmd = rospy.Publisher("image_echo", Image, queue_size = 10)
        self.found_green_light = rospy.Publisher("found_green_light", Bool, queue_size = 10)
        
        self.bridge = CvBridge()

        self.boxparam = np.array([0,0,0,0])
	self.maxcontour = []

    def proc_img(self, image):
        self.original = self.bridge.imgmsg_to_cv2(image,"bgr8")
        cv_image = self.original.copy()

        HSV = self.convertToHSV(cv_image)
        
        thresh = self.thresholdImage(HSV, self.TUNABLE_THRESH_GREEN)
        contours = self.getContours(thresh)
        
        # good_contours = contours
        good_contours = self.filterContours(contours)

        if len(good_contours) > 0:
            self.found_green_light.publish(True)
        else:
            self.found_green_light.publish(False)

        cv2.drawContours(cv_image, good_contours, -1, (0, 255, 0), thickness=3)

        #self.drawBox(img, contours)

        topub = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
        self.vid_cmd.publish(topub)
        
        #print "triggered"

    def convertToHSV(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def thresholdImage(self, image, threshold_array):
        lower_range = threshold_array[0]
        upper_range = threshold_array[1]
        mask = cv2.inRange(image, lower_range, upper_range)
        eroded = cv2.erode(mask, (3, 3), iterations=3)
        dilated = cv2.dilate(eroded, (3, 3), iterations=3)

        return dilated

    def getContours(self, image):
        ret, thresh = cv2.threshold(image, 127, 255, 0)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        return contours

    def filterContours(self, cnts):
        good_contours = []
        for c in cnts:
            # found green start light
            if cv2.contourArea(c) > self.TUNABLE_CONTOUR_AREA:
                good_contours.append(c)
        
        return good_contours

    def sortContours(self, cnts):
        # sort contours by area
        sorted_cnts = sorted (cnts, key=cv2.contourArea, reverse=True) 
        
        return sorted_cnts

    def drawBox(self, image, contours):
        a,b,c,max_h = cv2.boundingRect(contours[0])
        
	self.max_contour = contours[0]
        for contour in contours:
            #print contour
            x,y,w,h = cv2.boundingRect(contour)
            if h > max_h:
                max_h = h
                self.max_contour = contour
        #print(self.maxcontour)
        xf,yf,wf,hf = cv2.boundingRect(self.max_contour)
	print(wf*hf)
        if wf*hf > self.TUNABLE_CONTOUR_AREA:
            self.found_green_light.publish(True)
        else:
            self.found_green_light.publish(False)

'''
        cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
        self.boxparam=[xf,yf,wf,hf]

        rec_msg = Quaternion()  
        rec_msg.x = xf
        rec_msg.y = yf
        rec_msg.z = wf
        rec_msg.w = hf
        self.rec_cmd.publish(rec_msg)   
'''

if __name__ == "__main__":
    rospy.init_node("green_light")
    node = echoNode()
    rospy.spin()

