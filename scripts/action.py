#!/usr/bin/env python3

'''
import argparse
import os
import platform
import sys
from pathlib import Path
import threading
import torch
import pyrealsense2 as rs
import numpy as np
import re
'''
import rospy
import actionlib
import math
import sys
from Omatcha_actions.msg import val
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint

area = 0
cx=0
cy=0
count=0
def callback(message):
	#print("area: %f, cx: %f, cy: %f" %(message.area,message.cx,message.cy))
	global area
	global cx
	global cy
	global count
	area = message.area
	cx = message.cx
	cy = message.cy
	count = message.count
	
class ArmJointTrajectoryExample(object):

    def __init__(self):
        self._client = actionlib.SimpleActionClient("/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)

        self.gripper_client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.gripper_goal = GripperCommandGoal()
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
    
    def setup(self):
        global point
        global goal
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
    def setup2(self, secs2, time, sleep):
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        point.time_from_start = rospy.Duration(secs=secs2)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(time))
        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        rospy.sleep(sleep)
    
    def go(self):
        global joint_values
        
        global area
        global count
        global cx
        global cy
	
        self.setup()
        self.gripper_goal.command.position = math.radians(80.0)
        joint_values =[0.0,0.0,0.0,0.0,0.0,0.0,0.0] 
        self.setup2(2.5,100.0,1.0)

    
    
        print("serch stanby")
        self.setup()
        self.gripper_goal.command.position = math.radians(80.0)
        joint_values =[math.radians(-90), math.radians(-20), 0.0,math.radians(-150), 0.0, math.radians(100), math.radians(-90)] 
        self.setup2(2.5,100.0,2.5)
        
        count=0
        cx=0
        
        for i in range(0,181,2):
        	
        	self.setup()
        	#self.gripper_goal.command.position = math.radians(90.0)
        	joint_values =[math.radians(i-90), math.radians(-20), 0.0,math.radians(-150), 0.0, math.radians(100), math.radians(-90)]
        	self.setup2(0.2,100.0,0.1)
        	
        	print(count)
        	print(cx)
        	if count>20 and (cx-320)**2<500:
        		break
        	# rad略: -53 -62 0 -57 0 -60 -90
		
        i=i-90
        self.setup()
        joint_values =[math.radians(i), math.radians(20), 0.0,math.radians(-100), 0.0, math.radians(-90), math.radians(-90)] 
        self.setup2(1.5,100.0,0.1)
        
        
        for p in range (0,90,3):
        	
        	self.setup()
        	joint_values =[math.radians(i), math.radians(20-p), 0.0,math.radians(-100+p*0.9), 0.0, math.radians(-90+p*0.35), math.radians(-90)] 
        	j2=20-p*0.9
        	j4=-100+p*0.9
        	j6=-90+p*0.1
        	self.setup2(0.3,100.0,0.1)
        	
        	if (cy-240)**2 < 400:
        		break
        
        if(area>30000):
        	barea=area
        	self.setup()
        	joint_values = [math.radians(i), math.radians(j2), 0.0, math.radians(j4), 0.0, math.radians(j6), math.radians(-80)]
        	self.setup2(0.5,100.0,0.5)
        	
        	if(barea-area<0):
        		print("ah")
        		self.setup()
        		joint_values = [math.radians(i), math.radians(j2), 0.0,math.radians(j4), 0.0, math.radians(j6), math.radians(-100)] 
        		self.setup2(0.5,100.0,0.2)
        		mem=-120
        				
        	else:
        		print("ahh")
        		mem=-60
        			
        else:
        	mem=-90
     	
        for w in range(0,26,2):
        	
        	self.setup()
        	joint_values =[math.radians(i), math.radians(j2-w*1.25) , 0.0, math.radians(j4-w*1.7), 0.0, math.radians(2.2*w+j6), math.radians(mem)] 
        	self.setup2(0.4,100.0,0.3)
        	n1=i
        	n2=j2-w*1.2
        	n3=0
        	n4=j4-w*1.8
        	n5=0
        	n6=1.8*w+j6
        	n7=mem
        
        self.setup()
        self.gripper_goal.command.position = math.radians(0.0)
        self.setup2(2.5,100.0,1.0)	
        
        #shakemove
        
        self.setup()
        joint_values = [math.radians(-30.0), math.radians(-10), 0.0, math.radians(-55), math.radians(90.0), math.radians(-60), math.radians(90)]
        self.setup2(2.5, 100.0, 0.5)
        
        self.setup()
        joint_values = [math.radians(-15.0), math.radians(-21), 0.0, math.radians(-120), math.radians(0.0), math.radians(60), math.radians(90)] 
        self.setup2(1.5, 100.0, 0.5)
        
        
        
        for count in range(30):
        
        	self.setup()
        	joint_values = [math.radians(-15.0), math.radians(-21), 0.0, math.radians(-120), math.radians(0.0), math.radians(60), math.radians(82)]  
        	self.setup2(0.1, 100.0, 0)
        	#global area
        	#print(area)
      
        	self.setup()
        	joint_values = [math.radians(-15.0), math.radians(-21), 0.0, math.radians(-120), math.radians(0.0), math.radians(60), math.radians(98)]  
        	self.setup2(0.1, 100.0, 0)
        
        self.setup()        
        joint_values = [math.radians(n1), math.radians(0), math.radians(n3), math.radians(n4), math.radians(n5), math.radians(n6), math.radians(n7)]
        self.setup2(3.0, 100.0, 1)	
        self.setup()        
        joint_values = [math.radians(n1), math.radians(n2), math.radians(n3), math.radians(n4), math.radians(n5), math.radians(n6), math.radians(n7)]
        self.setup2(3.0, 100.0, 1)
        
        self.gripper_goal.command.position = math.radians(90)
      
        effort  = 1

        self.gripper_goal.command.max_effort = effort

     

        self.setup()        
        joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #角度指定部
        self.setup2(3.0, 100.0, 1)
        
        print("end of contents")

        return self._client.get_result()

    def feedback(self,msg):
        print("feedback callback")
        
def main():
    global completed
    completed = False
 
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    print("GO")
    if completed:
        pass
    else:
        arm_joint_trajectory_example.go()

        
if __name__ == '__main__':
    #opt = parse_opt()
    rospy.init_node("action")
    sub = rospy.Subscriber('num', val, callback)
    main()
    rospy.spin()
    
  
