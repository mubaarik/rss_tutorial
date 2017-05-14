#!/usr/bin/env python 
#import the ros client Python client, check https://wiki.ros.org/rospy
import rospy 
#import message type to publish
from sensor_msgs.msg import LaserScan 
#We will use the range 
from turtlesim.msg import Pose
#import numpy functions for calculations
import numpy as np

class TurtleSimScanner:
	def __init__(self):
		self.mat_width = 10;
		self.mat_heigh = 10;
		self.turtlePose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.poseCallback, queue_size = 1)
		self.scan_pub = rospy.Publisher("turtle_scan", LaserScan, queue_size=1)
		self.numb_ranges = 100
		self.min_angle = -np.pi/2.0
		self.max_angle = np.pi/2.0
		self.incr_angle = (self.max_angle-self.min_angle)/self.numb_ranges
		self.max_range = 4;
	'''
	@pose - position, direction, and velocity of the turtle recieved from the topic /turtle1/pose
	publishes a laser scan center around the turtle
	'''
	def poseCallback(self, pose):
		pose_data = pose.pose_data
		scan_msgs = LaserScan()

		scan_msgs.Header.stamp = rospy.now()
		scan_msgs.Header.frame_id = "base_link"
		scan_msgs.angle_min = min_angle
 		scan_msgs.angle_max = max_angle
		scan_msgs.angle_increment = incr_angle
		scan_msgs.time_increment = 0
		scan_msgs.scan_time = rospy.now()
		scan_msgs.range_min = .1
		scan_msgs.range_max = max_range
		scanMapFunc = np.vectorize(self.scanMap)
		scan_msgs.ranges = scanMapFunc(pose_data, range(self.numb_ranges))
		scan_msgs.intensities = np.ones(self.numb_ranges)

		scan_pub.publish(scan_msgs)
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

		if range_theta>7*np.pi/4.0 and range_theta<7*np.pi/4.0:
			slope = abs(range_theta - 3*np.pi/2)
			length = (self.height - pose_data.y)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(max_range,length)
			return index_range
		if range_theta=>3*np.pi/4.0 and range_theta<5*np.pi/4.0:
			slope = abs(range_theta - np.pi)
			length = (pose_data.x)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(max_range,length)
			return index_range
		if range_theta=>np.pi/4.0 and range_theta<3*np.pi/4.0:
			slope = abs(range_theta - np.pi/2.0)
			length = (pose_data.y)*np.sin(np.pi/2.0)/np.sin(slope)
			index_range = min(max_range,length)
			return index_range
		slope = abs(min(range_theta,2*np.pi-range_theta))
		length = (self.mat_width - pose_data.x)*np.sin(np.pi/2.0)/np.sin(slope)
		index_range = min(max_range,length)
		return index_range


	'''
	@angle - angle in radians
	@return - 0<=angle in radians<=2*pi -- representing counter-clockwise of this angle
	'''
	def positiveDirection(self, angle):
		theta = (angle%(2*np.pi)+2*np.pi)%(2*np.pi)
		return theta;


if __name__ == '__main__':
	rospy.init_node("TurtleSimScanPublisher")

	node = TurtleSimScanner()

	rospy.spin()

