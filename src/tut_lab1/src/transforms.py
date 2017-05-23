#!/usr/bin/env python 
#import the ros client Python client, check https://wiki.ros.org/rospy
import rospy 
#We will use the turtle position 
from turtlesim.msg import Pose
#import numpy functions for calculations
import numpy as np
#import tf
import tf 
class TransformPublisher:
	def __init__(self):
		
		self.turtlePose_sub=rospy.Subscriber("/turtle1/pose", Pose, self.poseCallback, queue_size = 1)
		print "initialzing node"
		#Coordinate transform publishers
		self.tf_pub = tf.TransformBroadcaster()
		self.world_pub = tf.TransformBroadcaster()
		self.frame = "turtle"
	def poseCallback(self,pose):
		
		self.worldToTurtleWorld()
		self.transform(pose)
	#Publish coordinate transform from world to the turtle mat 
	def worldToTurtleWorld(self):
		self.world_pub.sendTransform((5.0, 5.0,0.0), tf.transformations.quaternion_from_euler(0,0,np.pi), rospy.Time.now(), "turtle_world", "world")
	#coordinate transform from turtle mat to turtle pose
	def transform(self, pose):
		theta = pose.theta
		self.tf_pub.sendTransform((pose.x, pose.y,0), tf.transformations.quaternion_from_euler(0,0,theta), rospy.Time.now(), self.frame, "turtle_world")
if __name__ == '__main__':
	rospy.init_node("TransformPublisher")
	node = TransformPublisher()
	rospy.spin()

