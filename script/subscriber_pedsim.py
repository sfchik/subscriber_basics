#!/usr/bin/env python
## A basic pedsim subscriber

import rospy
import tf
import math
from pedsim_msgs.msg import TrackedPersons

class SubscriberPedsim:
	def __init__(self):
		self.r_ = rospy.Rate(10)

		#initializing message
		self.msg_tracked_persons_ = TrackedPersons()

		#Subscriber setup
		rospy.Subscriber('/pedsim/tracked_persons', TrackedPersons, self.callbackTrackedPersons)	

		#Dummy flag
		self.FIRST_MSG_ = False

	def callbackTrackedPersons(self, msg):
		self.msg_tracked_persons_ = msg
		self.FIRST_MSG_ = True

	def printMsg(self, msg):
		print msg 

	def getEularFromQuat(self, degree=True):
		quaternion = [self.msg_tracked_persons_.tracks[0].pose.pose.orientation.x, \
						self.msg_tracked_persons_.tracks[0].pose.pose.orientation.y, \
						self.msg_tracked_persons_.tracks[0].pose.pose.orientation.z, \
						self.msg_tracked_persons_.tracks[0].pose.pose.orientation.w] 
		eular = tf.transformations.euler_from_quaternion(quaternion) #convert quarternion data into eular	
		if degree:
			eular_degree = []
			for i in eular:
				eular_degree.append(180.0*i/math.pi)
			return eular_degree
		else:
			return eular

	def loop(self):
		while not rospy.is_shutdown():
			if self.FIRST_MSG_ == True:
				self.printMsg(self.getEularFromQuat())
			self.r_.sleep()

if __name__=="__main__":
	rospy.init_node('rl_pedsim_environment')	
	track_p = SubscriberPedsim()
	try:
		track_p.loop()
	except:
		pass