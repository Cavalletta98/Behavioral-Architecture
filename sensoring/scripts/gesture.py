#!/usr/bin/env python3

"""
    ROS node used to generate pointed gestures
"""

# Import of libraries
import rospy
from geometry_msgs.msg import Point
import random

## x coordinate of the map
map_x = rospy.get_param("map_x")

## y coordinate of the map
map_y = rospy.get_param("map_y")

## 2D Person position
person_pos = Point(rospy.get_param("person_pos_x"),rospy.get_param("person_pos_y"),0)

## Min delay for gesture generation
min_delay_gesture = rospy.get_param("min_delay_gesture")

## Max delay for gesture generation
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
        gesture_point.x = random.randint(0,map_x)
        gesture_point.y = random.randint(0,map_y)
        if (gesture_point.x != person_pos.x) and (gesture_point.y != person_pos.y):
            rospy.loginfo(gesture_point)
            pub.publish(gesture_point)
            rospy.sleep(random.uniform(min_delay_gesture,max_delay_gesture))

if __name__ == '__main__':
    try:
        gesture_generator()
    except rospy.ROSInterruptException:
        pass