#! /usr/bin/python

import rospy
import math


from baseMove import *

rospy.init_node('calibrate_base_node', anonymous=True)


# rospy.sleep(10)
# rate = rospy.Rate(10	.0)
# 

destination = [0,0]
angle = 0

bm = baseMove(verbose=True)


bm.goAngle(angle)
bm.goPosition(destination)
