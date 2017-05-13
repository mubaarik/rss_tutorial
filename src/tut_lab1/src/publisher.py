#!/usr/bin/env python 
import rospy 
from std_msgs.msg import Float32
import random as rd
class RandomAgePublisher:
	def __init__(self):
		self.pub = rospy.Publisher("/randomAge", Float32, queue_size = 1)
		self.age_range = 100
		print("initialized")

		self.publishAge()
	def publishAge(self):
		while not rospy.is_shutdown():
			age = float(rd.choice(range(self.age_range)))
			print(age)
			self.pub.publish(age)


def main():
	rospy.init_node("RandomAgePublisher")
	RandomAgePublisher()
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("RandomAgePublisher")
	node = RandomAgePublisher()
	rospy.spin()