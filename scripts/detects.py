#!/usr/bin/env python3

import rospy
from Omatcha_actions.msg import val

def callback(message):
	print("area: %f, cx: %f, cy: %f" %(message.area,message.cx,message.cy))
	
if __name__ == "__main__":
	rospy.init_node('detects_data')
	sub = rospy.Subscriber('num', val, callback)
	rospy.spin()
