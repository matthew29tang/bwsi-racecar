import rospy
import numpy as np
import time

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String
from time import sleep

import time

if __name__ == "__main__":

    rospy.init_node("wall_tester")
    
    wallPub = rospy.Publisher("/which_wall",
                                    String,
                                    queue_size=1)
    start_time = time.time()

    wall = "right"
    while not rospy.is_shutdown():
        wallPub.publish(wall)
        time_now = time.time()

        #print ("time_now = {}, start_time = {}".format(time_now, start_time))
        if time_now - start_time > 3:
            #print ("change")
            start_time = time.time()
            if wall == "right":
                wall = "left"
            else:
                wall = "right"
 
    rospy.spin()
