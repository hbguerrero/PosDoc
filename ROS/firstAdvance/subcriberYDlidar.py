#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callBack(message):
   print("Grade: %f " %message.data)

def callBack1(message):
   print("Range: %f " %message.data)

rospy.init_node("listener", anonymous = True)

rospy.Subscriber("ydlidarGrade", Float32, callBack)
rospy.Subscriber("ydlidarRange", Float32, callBack1)

rospy.spin()
