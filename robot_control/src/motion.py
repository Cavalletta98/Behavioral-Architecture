#!/usr/bin/env python3

"""
    ROS node used to simulate robot motion
"""

from robot_control.srv import TargetPos,TargetPosResponse
import rospy
import time
import random

min_delay_robot_motion = rospy.get_param("min_delay_robot_motion")
max_delay_robot_motion = rospy.get_param("max_delay_robot_motion")

def handle_target_pos(req):
    time.sleep(random.uniform(min_delay_robot_motion,max_delay_robot_motion))
    resp = TargetPosResponse()
    resp.feedback = "arrived"
    return resp

def target_pos_server():
    rospy.init_node('motion_server')
    s = rospy.Service('target_pos', TargetPos, handle_target_pos)
    rospy.spin()

if __name__ == "__main__":
    target_pos_server()