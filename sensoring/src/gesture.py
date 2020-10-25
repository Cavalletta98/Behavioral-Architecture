#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from random import randint


def gesture_generator():

    pub = rospy.Publisher('gesture', Point, queue_size=10)
    rospy.init_node('gesture_node', anonymous=True)
    #rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        gesture_point = Point()
        gesture_point.x = 1
        gesture_point.y =1
        rospy.loginfo(gesture_point)
        pub.publish(gesture_point)
        rospy.sleep(randint(1,10))
        #rate.sleep()

if __name__ == '__main__':
    try:
        gesture_generator()
    except rospy.ROSInterruptException:
        pass