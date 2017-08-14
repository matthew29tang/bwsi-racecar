#!/usr/bin/python

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Quaternion, Point

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from tuning_vars import TUNING_VARS

class FindYellow():
	def __init__(self):
		# TUNABLE_THRESH_YELLOW = [lowerhsv, upperhsv]
		self.TUNABLE_THRESH_YELLOW = TUNING_VARS.find_yellow_line_THRESH_YELLOW 
		#[np.array([18, 216, 123]), np.array([103,255,255])]
                self.TUNABLE_THRESH_ORANGE = TUNING_VARS.find_yellow_line_THRESH_ORANGE
		#[np.array([132, 106, 167]), np.array([38, 255, 255])]

		#TUNABLE_CONTOUR_AREA = minimumarea
        	self.TUNABLE_CONTOUR_AREA = TUNING_VARS.find_yellow_line_COUTOUR_AREA
		#500, edit this

		self.bridge = CvBridge()

                self.imgSub = rospy.Subscriber("/zed/rgb/image_rect_color",
                                  Image,
                                  self.find_yellow_line)

                self.vid_cmd = rospy.Publisher("/image_echo",
                                                Image,
                                                queue_size = 10)
		self.rec_cmd = rospy.Publisher("/contour_rectangle",
                                                Quaternion,
                                                queue_size = 10)
                self.center_cmd = rospy.Publisher("/cnt_center",
                                                   Point,
                                                   queue_size = 10)
                self.foundYellow_cmd = rospy.Publisher("/found_yellow",
                                                        Bool,
                                                        queue_size=10)
                # Constants for cv
                self.cut_size = TUNING_VARS.find_yellow_line_CUT_SIZE
		#2.0/3 # 1.0/2

                self.cx = 0
                self.cy = 0

		self.original = 0
		self.boxparam = np.array([0,0,0,0])

        def unsub_nodes(self):
                self.imgSub.unregister()

	def find_yellow_line(self, image):
 		# print ("Finding yellow")

                cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
		cut_image = self.cutImage(cv_image)
		self.original = cut_image

		HSV = self.convertToHSV(cut_image)

		thresh = self.thresholdImage(HSV, self.TUNABLE_THRESH_YELLOW)
		contours = self.getContours(thresh)
		good_contours = self.filterContours(contours)

		if len(good_contours) > 0:
                        sorted_cnts = self.sortContours(good_contours)
                        largest_cnt = sorted_cnts[0]
			self.cx, self.cy = self.centerOfContour(largest_cnt)
                        #cv2.drawContours(self.original, [largest_cnt], -1, (0,255,0), thickness=3)
                        #cv2.circle(self.original, (self.cx, self.cy), 20, (0, 255, 0), 3)
			#print("Contour found")
                        self.foundYellow_cmd.publish(True)
		else:
			# print("No valid contour found.")
                        self.foundYellow_cmd.publish(False)


		proc_img = self.bridge.cv2_to_imgmsg(self.original,"bgr8")
		self.vid_cmd.publish(proc_img)

                # publish contour center
                center = Point() # z is 0
                center.x = self.cx
                center.y = self.cy
                self.center_cmd.publish(center)

	def cutImage(self, image):
                h, w, channels = image.shape
                bottom_of_image = image[int(self.cut_size*h):h, :, :]
                return bottom_of_image

	def convertToHSV(self, image):
		return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	def thresholdImage(self, image, threshold_array):
		lower_range = threshold_array[0]
		upper_range = threshold_array[1]
		mask = cv2.inRange(image, lower_range, upper_range)
		return mask

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
                sorted_cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
                return sorted_cnts

        # uses moments to find centroid of contour
        def centerOfContour(self, cnt):
                M = cv2.moments(cnt)
                cx = 0
                cy = 0
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                return cx, cy

	def drawBox(self, contours):
		color = (255, 255,0)
		scale = 3
		a,b,c,maxh = cv2.boundingRect(contours[0])
		maxcontour = contours[0]
		for contour in contours:
			#print contour
    			x,y,w,h = cv2.boundingRect(contour)
			if h > maxh:
				maxh = h
				maxcontour = contour

		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
    		# cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.boxparam = [xf,yf,wf,hf]

		rec_msg = Quaternion()
		rec_msg.x = xf
		rec_msg.y = yf
		rec_msg.z = wf
		rec_msg.w = hf
		self.rec_cmd.publish(rec_msg)

	def drawText(self, image):
		color = (0,255,0)
		x = self.boxparam[0]
		y = self.boxparam[1]
		w = self.boxparam[2]
		h = self.boxparam[3]
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 1
		linetype = 4   # connected line
		text = str(w)
		point  = (x,y-20)  # x and y are integers
		cv2.putText(self.original,text,point,font, fontScale,color,linetype)
		cv2.putText(self.original,str(x-(self.original.shape[1]/2)),(x+100, y-20), font, fontScale, (255, 0, 0), linetype)

if __name__ == "__main__":
    rospy.init_node("find_yellow_line")
    node = FindYellow()
    rospy.spin()

