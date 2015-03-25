#! /usr/bin/python

import rospy
import moveit_commander
from geometry_msgs.msg import Twist
import tf
from numpy import linalg as LA
from math import *


# assuming there is already a ros node, do not init one here

# P control is sufficient for this function
class baseMove:
	def __init__(self, verbose=False):
		self.base_pub = rospy.Publisher('/base_controller/command', Twist)
		self.listener = tf.TransformListener()
		self.verbose = verbose
		self.posTolerance = 0.001
		self.angTolerance = 0.001
		self.r = rospy.Rate(100)

	def setPosTolerance(t):
		self.posTolerance = t

	def setAngTolerance(t):
		self.angTolerance = t

	def goPosition(self, destination): # translation only
		s = Twist()
		while True:
			try:
				(trans,rot) = self.listener.lookupTransform("/odom_combined", "/base_link", rospy.Time(0))
				theta = tf.transformations.euler_from_quaternion(rot)[2]
				x_diff = (destination[0] - trans[0])
				y_diff = (destination[1] - trans[1])
				alpha = atan2(y_diff, x_diff)
				r = alpha - theta
				if self.verbose:
					print 'X: %4f, Y: %4f' % (trans[0], trans[1])
				l = LA.norm([x_diff, y_diff])
				s.linear.x = l * cos(r) * 10
				s.linear.y = l * sin(r) * 10
				self.base_pub.publish(s)
				if LA.norm([x_diff, y_diff]) < self.posTolerance:
					if self.verbose:
						print 'destination arrived'
					return True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				self.r.sleep()

	def goAngle(self, angle):

		s = Twist()
		while True:
			try:
				(trans,rot) = self.listener.lookupTransform("/odom_combined", "/base_link", rospy.Time(0))
				theta = tf.transformations.euler_from_quaternion(rot)[2]
				if self.verbose:
					print 'theta: %4f, angle: %4f' % (theta, angle)
				z_diff = (angle - theta)
				s.angular.z = z_diff * 10
				self.base_pub.publish(s)
				if abs(z_diff) < self.angTolerance:
					if self.verbose:
						print 'angle arrived'
					return True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				rospy.sleep(0.1)
			self.r.sleep()