#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Quaternion

class follow_yellow_line:
	def __init__(self):

		###########################################
		
		# TUNABLE_THRESH_YELLOW = [lowerhsv, upperhsv]
		self.TUNABLE_THRESH_YELLOW = [np.array([27,150, 82]), np.array([32, 216, 255])]
		
		#TUNABLE_CONTOUR_AREA = minimumarea
        	self.TUNABLE_CONTOUR_AREA = 20 #edit this
		
		###########################################

		rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.proc_img)
		self.vid_cmd = rospy.Publisher("image_echo", Image, queue_size = 10)
		self.rec_cmd = rospy.Publisher("/contour_rectangle", Quaternion, queue_size = 10)

                self.cut_size = 2.0/3

		self.bridge = CvBridge()
		self.original = 0
		self.boxparam = np.array([0,0,0,0])

	def proc_img(self, image):
		cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
		cut_image = self.cutImage(cv_image)
		self.original = cut_image

		HSV = self.convertToHSV(cut_image)
		
		thresh = self.thresholdImage(HSV, self.TUNABLE_THRESH_YELLOW)
		contours = self.getContours(thresh)
		good_contours = self.filterContours(contours)

		if len(good_contours) > 0:
			self.drawBox(good_contours)
			print("Contour found")
		else:
			print("No valid contour found.")	
		
                self.drawText(self.original)

		topub=self.bridge.cv2_to_imgmsg(self.original,"bgr8")
		self.vid_cmd.publish(topub)
	
	def cutImage(self, image):
		h, w, channels = image.shape
                bottom_of_image = image[int(self.cut_size*h):h, :, :]
                return bottom_of_image
                #return cv2.rectangle(image, (0,0),(int(round(image.shape[1])),int(round(image.shape[0]*.7))), (0,0,0), -1)

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

	def drawBox(self, contours):
		color = (255,0,0)
		scale = 3
		a,b,c,maxh=cv2.boundingRect(contours[0])
		maxcontour=contours[0]
		for contour in contours:
			#print contour
    			x,y,w,h = cv2.boundingRect(contour)
			if h>maxh:
				maxh=h
				maxcontour=contour
		
		xf,yf,wf,hf = cv2.boundingRect(maxcontour)
    		cv2.rectangle(self.original, (xf,yf), (xf+wf,yf+hf), color, scale)
		self.boxparam=[xf,yf,wf,hf]

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
    rospy.init_node("follow_yellow_line")
    node = follow_yellow_line()
    rospy.spin()

