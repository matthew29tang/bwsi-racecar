import rospy

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class mesh_control:
    def __init__(self):
        # Subscribe to laser scan
        rospy.Subscriber("/scan",
                          LaserScan,
                          self.laserscan_callback)

        # Publish which side to use
        self.cmd_pub = rospy.Publisher("/which_wall",
                                        String, 
                                        queue_size=10)
       
    def laserscan_callback(self, scan):
        ranges = scan.ranges
        mid_index = len(ranges) / 2
    
        num_left_undefined = 0
        num_right_undefined = 0
        search_range = 360
        exclude_range = 180
        #print ranges

        left_sum = 0
        right_sum = 0

        # split scan into two sections, left and right
        for i in range(mid_index - search_range, mid_index - 180):
            dist = ranges[i]
            right_sum += dist
        for j in range(mid_index + 180, mid_index + search_range):
            dist = ranges[j]
            left_sum += dist

        if left_sum < right_sum:
            wall_side = "left"
        else:
            wall_side = "right"
        
        print (wall_side)
        # print ("right = {}, left = {}".format(right_sum, left_sum))        
        self.cmd_pub.publish(String(wall_side))

       # for i in range(len(ranges)):
       #     dist = ranges[i]
       #     if dist > 5: # laser did not detect anything (light doesn't return or takes too long)
       #         
       #         if i < mid_index-search_range: # count the # of undefined pts on the right
       #             #print dist
       #             num_right_undefined += 1; 
       #         elif i > mid_index+search_range: # count the # of undefined pts on the left
       #             num_left_undefined += 1

       # print ("right most = ", ranges[mid_index-search_range])

       # #print "LEFT: " , num_left_undefined
       # #print "RIGHT: ", num_right_undefined
       # 
       # if num_left_undefined < num_right_undefined:
       #     wall_side = "left"
       #     #print "LEFT"
       # else:
       #     wall_side = "right"
       #     #print "RIGHT"
       # self.cmd_pub.publish(wall_side)

if __name__ == '__main__':
    rospy.init_node("mesh_control")
    node = mesh_control()
    rospy.spin()
