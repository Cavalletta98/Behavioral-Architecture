#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from random import randint

def command_generator():

    pub = rospy.Publisher('command', String, queue_size=1)
    rospy.init_node('command_node', anonymous=True)
    #rate = rospy.Rate(randint(1,10)) # 10hz

    while not rospy.is_shutdown():
        command = "play"
        rospy.loginfo(command)
        pub.publish(command)
        #rate.sleep()
        rospy.sleep(randint(1,10))

if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass