#! /usr/bin/python

import rospy
import math


from baseMove import *

rospy.init_node('calibrate_base_node', anonymous=True)


# rospy.sleep(10)
# rate = rospy.Rate(10	.0)
# 

position = [0,0]
angle = 0

bm = baseMove(verbose=False)

bm.setPosTolerance(0.001)
bm.setAngTolerance(0.001)
bm.setLinearGain(10)
bm.setAngularGain(10)


bm.go(position, angle)

# bm.goAngle(angle)
# bm.goPosition(destination)
# bm.goAngle(angle)