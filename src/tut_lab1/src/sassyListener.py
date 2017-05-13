#!/usr/bin/env python 
#import the ros client Python client, check https://wiki.ros.org/rospy
import rospy 
#import the message type published by the publisher
from std_msgs.msg import Float32
#import the message type to publish
from std_msgs.msg import String
class Listener:
	def __init__(self):
		#Listen to Float32 message type on the topic /randomAge
		#Note: the use of the callback function, ageCallback
		#This callback will be called everytime this topic is published on
		self.listen = rospy.Subscriber("/randomAge", Float32, self.ageCallback, queue_size =1)
		self.pub = rospy.Publisher("/sassComments", String, queue_size = 1)
	def ageCallback(self, age):
		age = age.data
		if age<15:
			self.pub.publish("You're too young to talk about age")
		elif age<30:
			self.pub.publish("You're young still!")
		elif age<50:
			self.pub.publish("Stop talking and rule the world")
		else:
			self.pub.publish("time to retire!")

if __name__ == '__main__':
	#initialized the node and give it a name
	#note that the class and the name of the node don't have to be the same
	rospy.init_node("sassyListener")
	#The class that implements the node
	node  = Listener()
	#loop
	rospy.spin()
