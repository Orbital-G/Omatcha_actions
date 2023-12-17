#!/usr/bin/env python3

import rospy
import threading
from Omatcha_actions.msg import val
area=0
def callback(message):
	#print("area: %f, cx: %f, cy: %f" %(message.area,message.cx,message.cy))
	global area
	area=message.area
def main():
	global area
	while True:
		print(area)
		
if __name__ == "__main__":
	rospy.init_node('detects_data')
	sub = rospy.Subscriber('num', val, callback)
	thread=threading.Thread(target=main)
	thread.start()
	rospy.spin()
	
