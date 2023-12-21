#!/usr/bin/env python3


import rospy
import actionlib
import math
import sys
from Omatcha_actions.msg import val2
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint

cx2 = 0
count = 0
depth = 0
def callback(message):
	print("%f" %(message.depth))
	global cx2
	global count2
	global depth
	cx2 = message.cx2
	count2 = message.count2
	depth = message.depth
	
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
        global count2
        global cx2
        global depth
	
        self.setup()
        self.gripper_goal.command.position = math.radians(80.0)
        joint_values =[0.0,0.0,0.0,0.0,0.0,0.0,0.0] 
        self.setup2(2.5,100.0,1.0)

    
    
        print("serch stanby")
        self.setup()
        self.gripper_goal.command.position = math.radians(80.0)
        joint_values =[math.radians(-90), math.radians(-20), 0.0,math.radians(-150), 0.0, math.radians(100), math.radians(90)] 
        self.setup2(2.5,100.0,2.5)
        
        count2=0
        for i in range(0,45,2):
        	
        	self.setup()
        	#self.gripper_goal.command.position = math.radians(90.0)
        	joint_values =[math.radians(i-90), math.radians(-20), 0.0,math.radians(-150), 0.0, math.radians(100), math.radians(90)]
        	self.setup2(0.2,100.0,0.1)
        	
        	#print(count)
        	print(depth)
        	if count2>20 and (cx2-320)**2<500:
        		break
        	# rad略: -53 -62 0 -57 0 -60 -90
        i=i-90
		
        for p in range(0,130,2):
        	self.setup()
        	joint_values = [math.radians(i), math.radians(-20-p*0.5), 0.0, math.radians(-150+p), 0.0, math.radians(90-p*0.4), math.radians(90)]
        	self.setup2(0.2,100,0.0)
        	
     	
        
        
        self.setup()
        self.gripper_goal.command.position = math.radians(0.0)
        self.setup2(1.5,100.0,1.0)	
        
        self.setup()
        joint_values = [math.radians(-15), math.radians(-21), 0.0, math.radians(-120), math.radians(0.0), math.radians(60), math.radians(90)] 
        self.setup2(1.5, 100.0, 0.5)
        
        
        
        
        
        print("pour")
        self.setup()
        joint_values = [math.radians(-18.0), math.radians(-21), math.radians(2), math.radians(-120), math.radians(0.0), math.radians(60), math.radians(90)]  
        self.setup2(0.1, 100.0, 0)
        
        self.setup()
        joint_values = [math.radians(-18.0), math.radians(-21), math.radians(2), math.radians(-120), math.radians(0.0), math.radians(60), math.radians(-75)]  
        self.setup2(0.5, 100.0, 3.0)
        	
        
      
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
    sub = rospy.Subscriber('num2', val2, callback)
    main()
    rospy.spin()
    
  
