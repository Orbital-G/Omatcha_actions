#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    rospy.loginfo("Received AR Marker ID: %d", msg.pose.position.x)

def test_ar_marker_subscriber():
    rospy.init_node('ar_marker_test_subscriber', anonymous=True)
    rospy.Subscriber('/ar_marker_id', PoseStamped, pose_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        test_ar_marker_subscriber()
    except rospy.ROSInterruptException:
        pass
