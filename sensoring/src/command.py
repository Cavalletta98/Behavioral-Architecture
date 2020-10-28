#!/usr/bin/env python3

"""
    ROS node used to generate user command
"""

import rospy
from std_msgs.msg import String
import random

min_delay_command = rospy.get_param("min_delay_command")
max_delay_command = rospy.get_param("max_delay_command")

def command_generator():

    """
        Main function that generates user command
        and publishes it on topic "command"
    """

    pub = rospy.Publisher('command', String, queue_size=1)
    rospy.init_node('command_node', anonymous=True)
    #rate = rospy.Rate(randint(1,10)) # 10hz

    while not rospy.is_shutdown():
        command = "play"
        rospy.loginfo(command)
        pub.publish(command)
        #rate.sleep()
        rospy.sleep(random.uniform(min_delay_command,max_delay_command))

if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass