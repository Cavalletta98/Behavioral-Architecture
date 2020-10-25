#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from random import randint

map_x = rospy.get_param("map_x")
map_y = rospy.get_param("map_y")

def gesture_generator():

    pub = rospy.Publisher('gesture', Point, queue_size=10)
    rospy.init_node('gesture_node', anonymous=True)
    #rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        gesture_point = Point()
        gesture_point.x = randint(1,map_x)
        gesture_point.y = randint(1,map_y)
        rospy.loginfo(gesture_point)
        pub.publish(gesture_point)
        rospy.sleep(randint(1,10))
        #rate.sleep()

if __name__ == '__main__':
    try:
        gesture_generator()
    except rospy.ROSInterruptException:
        pass