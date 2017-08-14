#!/usr/bin/python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool

class GreenLight:
    def __init__(self):
        # TUNABLE_THRESH_GREEN = [lowerhsv, upperhsv]
        self.TUNABLE_THRESH_GREEN = [np.array([1, 150, 120]),
                                     np.array([20,255,255])]

        #TUNABLE_CONTOUR_AREA = minimumarea
        self.TUNABLE_CONTOUR_AREA = 200 #edit this

        # Subscribe to zed_images
        self.imgSub = rospy.Subscriber("/zed/rgb/image_rect_color",
                          Image,
                          self.find_green_light)

        self.vid_cmd = rospy.Publisher("/image_echo",
                                        Image,
                                        queue_size = 10)

        self.found_green_pub = rospy.Publisher("/found_green",
                                                Bool,
                                                queue_size = 1)

        self.found_green_light = False

        self.bridge = CvBridge()

    def unsub_nodes(self):
        self.imgSub.unregister()

    def find_green_light(self, image):
        print ("Finding green")

        self.original = self.bridge.imgmsg_to_cv2(image,"bgr8")
        cv_image = self.original.copy()

        HSV = self.convertToHSV(cv_image)

        thresh = self.thresholdImage(HSV, self.TUNABLE_THRESH_GREEN)
        contours = self.getContours(thresh)

        # good_contours = contours
        good_contours = self.filterContours(contours)

        if len(good_contours) > 0:
            self.found_green_light = True

        self.found_green_pub.publish(self.found_green_light)

        #cv2.drawContours(cv_image, good_contours, -1, (0, 255, 0), thickness=3)

        #self.drawBox(img, contours)

        #topub = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
        #self.vid_cmd.publish(topub)

    def convertToHSV(self, image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def thresholdImage(self, image, threshold_array):
        lower_range = threshold_array[0]
        upper_range = threshold_array[1]
        mask = cv2.inRange(image, lower_range, upper_range)
        # remove noise
        eroded = cv2.erode(mask, (3, 3), iterations=3)
        dilated = cv2.dilate(eroded, (3, 3), iterations=3)

        return dilated

    def getContours(self, thresh):
        # ret, thresh = cv2.threshold(image, 127, 255, 0)
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

if __name__ == "__main__":
    rospy.init_node("green_light")
    node = GreenLight()
    rospy.spin()

