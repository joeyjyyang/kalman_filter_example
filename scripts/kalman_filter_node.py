#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

class KalmanFilter:
   	def __init__(self):		   
		# Create subscribers to IMU and UWB Tag topics.
		# The callback methods imuCallback() and uwbTagCallback() will be triggered
		# everytime a new message is received by this node on the IMU and UWB Tag topics.
		self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback, queue_size = 1)
		self.uwb_tag_sub = rospy.Subscriber("/tag", Marker, self.uwbTagCallback, queue_size = 1)

		# Member variables to hold the IMU and UWB messages received.
		# It's useful to store these messages locally as member variables since
		# they can be processed in/by other methods.
		# For now, they are initialized as empty Imu() and Marker() messages.
		self.imu_msg = Imu()
		self.uwb_tag_msg = Marker()

		# Initialize some Kalman Filter related things,
		# such as state, covariance matrices, etc.
		self.initState()
		self.initCovar()

	def imuCallback(self, imu_msg):
		# Update the member variable to store newest IMU message.
		self.imu_msg = imu_msg
		
		# 
		# Do some post-processing here if you want,
		# or just print the message: print(imu_msg)
		#

	def uwbTagCallback(self, uwb_tag_msg):
		# Update the member variable to store newest UWB Tag message.
		self.uwb_tag_msg = uwb_tag_msg
		
		# 
		# Do some post-processing here if you want,
		# or just print the message: print(uwb_tag_msg)
		#

	def initState(self):
		# This method implements the initialization of the states
		# for the system.
		pass

	def initCovar(self):
		# This method implements the initialization of the covariance matrices
		# for the system.
		pass

	def kalmanPredict(self):
		# This method implements the prediction step of the Kalman Filter.

		# 
		# You will possibly need to do something with the IMU and UWB data here,
		# which can be accessed with the self.imu_msg and self.uwb_tag_msg
		# member variables.
		#

		pass

	def kalmanUpdate(self):
		# This method implements the update/correction step of the Kalman Filter.
		
		# 
		# You will possibly need to do something with the IMU and UWB data here,
		# which can be accessed with the self.imu_msg and self.uwb_tag_msg
		# member variables.
		#

		pass

if __name__ == '__main__':
	# Create and register this ROS node with the ROS master.
	rospy.init_node("kalman_filter_node")

	rospy.loginfo("Initialized kalman_filter_node")

	# Create a Kalman Filter object.
	kalman_filter = KalmanFilter()

	# Run the node forever (or until it is killed with CTRL + C).
	try:
		rospy.spin() 
	except rospy.ROSInterruptException:
		pass