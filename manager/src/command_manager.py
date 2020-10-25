#!/usr/bin/env python3

import roslib
import rospy
import smach
import smach_ros
import time
import random
from geometry_msgs.msg import Point
from std_msgs.msg import String

home_pos = Point()
home_pos.x = 5
home_pos.y = 5

person_pos = Point()
person_pos.x = 8
person_pos.y = 10

pub_target_pos = rospy.Publisher('target_position', Point, queue_size=10)

# define state Play
class play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['someTimes'])
        self.arrived = 0
        self.transition = 0
        self.count = 0

    def getFeedback(self,data):
        self.arrived = 1
    
    def getGesture(self,data):
        pub_target_pos.publish(data)
        while self.arrived == 0:
            pass
        self.arrived = 0
        pub_target_pos.publish(person_pos)

        while self.arrived == 0:
            pass
        self.arrived = 0
        self.count += 1
        if self.count == 5:
            self.transition = 1

        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state PLAY')

        rospy.Subscriber("feedback", String, self.getFeedback)

        pub_target_pos.publish(person_pos)

        while self.arrived == 0:
            pass

        self.arrived = 0
        rospy.Subscriber("gesture", Point, self.getGesture)

        while self.transition == 0:
            pass
        return 'someTimes'

# define state Sleep
class sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])
        self.arrived = 0

    def getFeedback(self,data):
        time.sleep(5)
        self.arrived = 1
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')

        rospy.Subscriber("feedback", String, self.getFeedback)

        pub_target_pos.publish(home_pos)
        
        while self.arrived == 0:
            pass

        return 'wakeUp'
    

# define state Normal
class normal(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['play','someTimes'])
        self.arrived = 0
        self.count = 0
        self.play = 0

    def getFeedback(self,data):
        self.arrived = 1

    def getCommand(self,data):
        self.play = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state NORMAL')

        rospy.Subscriber("command", String, self.getCommand)
        rospy.Subscriber("feedback", String, self.getFeedback)

        while self.count < 2:
            if self.play == 1:
                return 'play'
            else:
                position = Point()
                position.x = 6+self.count
                position.y = 21+self.count
                pub_target_pos.publish(position)

                while self.arrived == 0:
                    pass

                self.count +=1

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