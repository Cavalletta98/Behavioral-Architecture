#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import time
from std_msgs.msg import String
import random

min_delay_robot_motion = rospy.get_param("min_delay_robot_motion")
max_delay_robot_motion = rospy.get_param("max_delay_robot_motion")

pub = rospy.Publisher('feedback', String, queue_size=10)

def getTargetPos(data):
    time.sleep(random.uniform(min_delay_robot_motion,max_delay_robot_motion))
    pub.publish("arrived")
 
def motion():

    rospy.init_node('motion_node', anonymous=True)

    rospy.Subscriber('target_position', Point, getTargetPos)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    motion()