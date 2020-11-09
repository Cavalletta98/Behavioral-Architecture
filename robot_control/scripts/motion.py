#!/usr/bin/env python3

"""
    ROS service used to simulate robot motion
"""

# Import of libraries
from robot_control.srv import TargetPos,TargetPosResponse
import rospy
import time
import random

## Min delay for simulate the robot motion
min_delay_robot_motion = rospy.get_param("min_delay_robot_motion")

## Max delay for simulate the robot motion
max_delay_robot_motion = rospy.get_param("max_delay_robot_motion")

def handle_target_pos(req):

    """
        Received a request (2D position), sleep for a random number fo seconds
        and reply with "arrvied"

        @param req: requested data
        @type req: TargetPosResponse
    """

    time.sleep(random.uniform(min_delay_robot_motion,max_delay_robot_motion))
    resp = TargetPosResponse()
    resp.feedback = "arrived"
    return resp

def target_pos_server():

    """
        Initialize the ROS server
    """

    rospy.init_node('motion_server')
    s = rospy.Service('target_pos', TargetPos, handle_target_pos)
    rospy.spin()

if __name__ == "__main__":
    target_pos_server()