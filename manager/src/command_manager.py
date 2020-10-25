#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
from random import randint
from geometry_msgs.msg import Point
from std_msgs.msg import String

home_pos = Point()
home_pos.x = rospy.get_param("home_pos_x")
home_pos.y = rospy.get_param("home_pos_y")

person_pos = Point()
person_pos.x = rospy.get_param("person_pos_x")
person_pos.y = rospy.get_param("person_pos_y")

map_x = rospy.get_param("map_x")
map_y = rospy.get_param("map_y")

pub_target_pos = rospy.Publisher('target_position', Point, queue_size=1)

# define state Play
class play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['someTimes'])
        self.arrived = 0
        self.transition = 0
        self.count = 0
        self.transition_value = randint(1,10)

    def getFeedback(self,data):
        self.arrived = 1
    
    def getGesture(self,data):
        if self.transition == 0:
            pub_target_pos.publish(data)
            while self.arrived == 0:
                pass
            print("Robot arrived in: ",data.x,data.y)
            self.arrived = 0
            pub_target_pos.publish(person_pos)

            while self.arrived == 0:
                pass
            print("Robot arrived in person position")
            self.arrived = 0
            self.count += 1
            if self.count == self.transition_value:
                self.transition = 1

        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        self.arrived = 0
        self.transition = 0
        self.count = 0
        self.transition_value = randint(1,10)
        rospy.loginfo('Executing state PLAY')

        sub_feedback = rospy.Subscriber("feedback", String, self.getFeedback)

        pub_target_pos.publish(person_pos)

        while self.arrived == 0:
            pass

        print("Robot arrived in person position")

        self.arrived = 0
        sub_gesture = rospy.Subscriber("gesture", Point, self.getGesture)

        while self.transition == 0:
            pass
        sub_feedback.unregister()
        sub_gesture.unregister()
        return 'someTimes'

# define state Sleep
class sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])
        self.arrived = 0

    def getFeedback(self,data):
        time.sleep(randint(1,10))
        self.arrived = 1
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        self.arrived = 0
        rospy.loginfo('Executing state SLEEP')

        sub_feedback = rospy.Subscriber("feedback", String, self.getFeedback)

        pub_target_pos.publish(home_pos)
        
        while self.arrived == 0:
            pass
        print("Robot arrived in home")
        sub_feedback.unregister()
        return 'wakeUp'
    

# define state Normal
class normal(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['play','someTimes'])
        self.arrived = 0
        self.play = 0

    def getFeedback(self,data):
        self.arrived = 1

    def getCommand(self,data):
        self.play = 1

    def execute(self, userdata):

        self.arrived = 0
        self.play = 0

        rospy.loginfo('Executing state NORMAL')

        sub_command = rospy.Subscriber("command", String, self.getCommand)
        sub_feedback = rospy.Subscriber("feedback", String, self.getFeedback)
        count_value = randint(1,10)

        for count in range(0,count_value):
            position = Point()
            position.x = randint(1,map_x)
            position.y = randint(1,map_y)
            pub_target_pos.publish(position)

            while self.arrived == 0:
                pass
            print("Robot arrived in: ",position.x,position.y)
            if self.play == 1:
                sub_command.unregister()
                return 'play'

        sub_feedback.unregister()
        return 'someTimes'

        
def main():
    rospy.init_node('command_manager_state_machine')
    

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', normal(), 
                               transitions={'play':'PLAY', 
                                            'someTimes':'SLEEP'})
        smach.StateMachine.add('SLEEP', sleep(), 
                               transitions={'wakeUp':'NORMAL'})
        smach.StateMachine.add('PLAY', play(), 
                               transitions={'someTimes':'NORMAL'})
        
        


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()