#!/usr/bin/env python 
#import the ros client Python client, check https://wiki.ros.org/rospy
import rospy 
#import message type to publish
from sensor_msgs.msg import LaserScan 
#We will use the range 
from turtlesim.msg import Pose
#import numpy functions for calculations
import numpy as np
#import tf
import tf 

class TurtleSimScanner:
	def __init__(self):
		self.mat_width = 11.0888891221;
		self.mat_height = 11.0888891221;
		self.turtlePose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.poseCallback, queue_size = 1)
		self.scan_pub = rospy.Publisher("turtle_scan", LaserScan, queue_size=1)
		self.numb_ranges = 1000
		self.min_angle = -np.pi/2.0
		self.max_angle = np.pi/2.0
		self.incr_angle = (self.max_angle-self.min_angle)/self.numb_ranges
		self.max_range = 2;
		self.tf_pub = tf.TransformBroadcaster()
		self.frame = "turtle"
	'''
	@pose - position, direction, and velocity of the turtle recieved from the topic /turtle1/pose
	publishes a laser scan center around the turtle
	'''
	def poseCallback(self, pose):
		pose_data = pose
		self.transform(pose_data)
		scan_msgs = LaserScan()

		scan_msgs.header.stamp = rospy.Time.now()
		scan_msgs.header.frame_id = self.frame
		scan_msgs.angle_min = self.min_angle
 		scan_msgs.angle_max = self.max_angle
		scan_msgs.angle_increment = self.incr_angle
		scan_msgs.time_increment = 0
		#scan_msgs.scan_time = rospy.Time.now()
		scan_msgs.range_min = .1
		scan_msgs.range_max = self.max_range
		scanMapFunc = np.vectorize(self.scanMap)
		scan_msgs.ranges = scanMapFunc(pose_data, range(self.numb_ranges)).tolist()
		scan_msgs.intensities = (100*np.ones(self.numb_ranges)).tolist()

		self.scan_pub.publish(scan_msgs)
	'''
	@pose_data - class representing position, heading, and velocity of the turtle
	@index - index to LaserScan().ranges, integer between 0, and num_ranges
	@return - minimum of the distance to the edge of the turtle mat and max_range
	'''
	def scanMap(self, pose_data, index):
		#get the counter-clockwise heading of the turtle
		bot_pos_theta = self.positiveDirection(pose_data.theta)
		#angle difference between this range and the turtle heading
		delta_theta = self.incr_angle*(index - self.numb_ranges/2.0)
		#get the counter-clockwise heading direction of this range
		range_theta = self.positiveDirection(bot_pos_theta+delta_theta)

		if range_theta>=7*np.pi/4.0 and range_theta<7*np.pi/4.0:
			slope = abs(range_theta - 3*np.pi/2)
			length = (pose_data.y)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(self.max_range,length)
			return index_range
		if range_theta>=3*np.pi/4.0 and range_theta<5*np.pi/4.0:
			slope = abs(range_theta - np.pi)
			length = (self.mat_width -pose_data.x)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(self.max_range,length)
			return index_range
		if range_theta>=np.pi/4.0 and range_theta<3*np.pi/4.0:
			slope = abs(range_theta - np.pi/2.0)
			length = (self.mat_height - pose_data.y)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(self.max_range,length)
			return index_range
		slope = abs(min(range_theta,2*np.pi-range_theta))
		length = (pose_data.x)*np.sin(np.pi/2.0)/np.sin(slope)
		index_range = min(self.max_range,length)
		return index_range


	'''
	@angle - angle in radians
	@return - 0<=angle in radians<=2*pi -- representing counter-clockwise of this angle
	'''
	def positiveDirection(self, angle):
		theta = (angle%(2*np.pi)+2*np.pi)%(2*np.pi)
		return theta;
	def transform(self, pose):
		self.tf_pub.sendTransform((pose.x, pose.y,0), tf.transformations.quaternion_from_euler(0,0,pose.theta), rospy.Time.now(), self.frame, "world")


if __name__ == '__main__':
	rospy.init_node("TurtleSimScanPublisher")

	node = TurtleSimScanner()

	rospy.spin()

