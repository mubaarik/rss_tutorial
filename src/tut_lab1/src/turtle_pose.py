#!/usr/bin/env python 
#import the ros client Python client, check https://wiki.ros.org/rospy
import rospy 
#import message type to publish
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseStamped
#We will use the turtle position
from turtlesim.msg import Pose as tpose
#import numpy functions for calculations
import numpy as np
#import tf
import tf 
'''
Publishes the robot position and orientation for rviz visualization
'''
class TurtlePose:
	def __init__(self):
		self.sub = rospy.Subscriber("/turtle1/pose", tpose, self.poseCallback, queue_size=1)
		self.pub = rospy.Publisher("pose", PoseStamped, queue_size=1)
	def poseCallback(self, msg):
		theta = (msg.theta)%(2*np.pi)
		_pose = PoseStamped()
		_pose.header.stamp = rospy.Time.now()
		_pose.header.frame_id = "turtle_world"

		pose = Pose()
		pose.position.x = msg.x
		pose.position.y = msg.y
		pose.position.z = 0
		x,y,z,w = tf.transformations.quaternion_from_euler(0,0,msg.theta)

		pose.orientation.x = x
		pose.orientation.y =y
		pose.orientation.z = z
		pose.orientation.w = w 

		_pose.pose = pose

		self.pub.publish(_pose)
if __name__ == '__main__':
	rospy.init_node("TurtlePose")

	node = TurtlePose()

	rospy.spin()

