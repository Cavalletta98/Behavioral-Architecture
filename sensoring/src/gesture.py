#!/usr/bin/env python3

"""
    ROS node used to generate pointed gestures
"""

import rospy
from geometry_msgs.msg import Point
import random

map_x = rospy.get_param("map_x")
map_y = rospy.get_param("map_y")

person_pos = Point()
person_pos.x = rospy.get_param("person_pos_x")
person_pos.y = rospy.get_param("person_pos_y")

min_delay_gesture = rospy.get_param("min_delay_gesture")
max_delay_gesture = rospy.get_param("max_delay_gesture")

def gesture_generator():

    """
        Main function that generates random pointed gestures 
        and publish them on topic "gesture"
    """

    pub = rospy.Publisher('gesture', Point, queue_size=10)
    rospy.init_node('gesture_node', anonymous=True)

    while not rospy.is_shutdown():
        gesture_point = Point()
        gesture_point.x = random.randint(1,map_x)
        gesture_point.y = random.randint(1,map_y)
        if (gesture_point.x != person_pos.x) and (gesture_point.y != person_pos.y):
            rospy.loginfo(gesture_point)
            pub.publish(gesture_point)
            rospy.sleep(random.uniform(min_delay_gesture,max_delay_gesture))

if __name__ == '__main__':
    try:
        gesture_generator()
    except rospy.ROSInterruptException:
        pass