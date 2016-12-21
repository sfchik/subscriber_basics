#!/usr/bin/env python
## A basic pedsim subscriber

import rospy
import tf
import math
from pedsim_msgs.msg import TrackedPersons
from people_msgs.msg import People
from people_msgs.msg import Person

class SubscriberPedsim:
	def __init__(self):
		self.r_ = rospy.Rate(100)

		#initializing message
		self.msg_tracked_persons_ = TrackedPersons()
		self.msg_people_ = People()
		self.msg_person_ = Person()

		#Subscriber setup
		rospy.Subscriber('/pedsim/tracked_persons', TrackedPersons, self.callbackTrackedPersons)

		#Publisher setup
		self.pub_people_ = rospy.Publisher('/people', People, queue_size=10)	

		#Dummy flag
		self.FIRST_MSG_ = False

	def callbackTrackedPersons(self, msg):
		self.msg_tracked_persons_ = msg
		self.FIRST_MSG_ = True

	# this function should not be used, refer to subscriber_pedsim_remap.cpp
	def remapToPeople(self):
		self.msg_people_.header = self.msg_tracked_persons_.header

		for i in self.msg_tracked_persons_.tracks:
			self.msg_person_.name = str(i.track_id)
			self.msg_person_.position = i.pose.pose.position
			self.msg_person_.velocity = i.twist.twist.linear
			self.msg_people_.people.append(self.msg_person_)

		self.pub_people_.publish(self.msg_people_)	


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
				# self.remapToPeople()
			self.r_.sleep()

if __name__=="__main__":
	rospy.init_node('rl_pedsim_environment')	
	track_p = SubscriberPedsim()
	try:
		track_p.loop()
	except:
		pass