#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def callback(message):
	print(message.data)
	
if __name__ == "__main__":
	rospy.init_node('detectr.data')
	sub = rospy.Subscriber('num', Int16, callback)
	rospy.spin()
