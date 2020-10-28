#!/usr/bin/env python3

"""
    ROS node that implement the FSM of 
    robot behaviours
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
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

min_transition_play_normal = rospy.get_param("min_transition_play_normal")
max_transition_play_normal = rospy.get_param("max_transition_play_normal")

min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

min_sleep_delay = rospy.get_param("min_sleep_delay")
max_sleep_delay = rospy.get_param("max_sleep_delay")

pub_target_pos = rospy.Publisher('target_position', Point, queue_size=1)

# define state Play
class play(smach.State):

    """
        A class used to represent the PLAY behaviour
        of the robot

        Attributes
        --------
        @param arrived: Advise if the robot reaches the position
        @type arrived: int
        @param transition: Advise if it is necessary to change state (NORMAL)
        @type transition: int
        @param count: count the number of times it is performed the PLAY behaviour
        @type count: int
        @param transition_value: define the value at wich it is performed the state transition
        @type transition_value: int

        Methods
        -----
        getFeedback(data)
            Callback method that received the "arrived" message and set the attribute
            arrived to 1
        getGesture(data)
            Callback method that receives the pointed gesture and perform the PLAY 
            behaviour
        execute()
            It publishes the person position and, after the robot reaches the position, it
            subsribes to "gesture" topic. At the end, it waits for the state transition
    """

    def __init__(self):
        # initialisation function, it should not wait

        """
            Constrcutor. It inizializes the attributes and subscribe to "feedback" topic
        """

        smach.State.__init__(self,outcomes=['someTimes'])
        self.arrived = 0
        self.transition = 0
        self.count = 0
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        rospy.Subscriber("feedback", String, self.getFeedback)
        
    def getFeedback(self,data):

        """
            Callback method that received the "arrived" message and set the attribute
            arrived to 1

            @param data: feedback message
            @type data: str
        """

        self.arrived = 1
    
    def getGesture(self,data):

        """
            Callback method that receives the pointed gesture and perform the PLAY 
            behaviour

            @param data: 2D pointed gesture
            @type data: Point
        """

        if self.transition == 0:
            pub_target_pos.publish(data)
            while self.arrived == 0:
                pass
            rospy.loginfo("Robot arrived in (%d,%d)",data.x,data.y)
            self.arrived = 0
            pub_target_pos.publish(person_pos)

            while self.arrived == 0:
                pass
            rospy.loginfo("Robot arrived in person position (%d,%d)",person_pos.x,person_pos.y)
            self.arrived = 0
            self.count += 1
            if self.count == self.transition_value:
                self.transition = 1

        
    def execute(self, userdata):

        """
            It publishes the person position and, after the robot reaches the position, it
            subsribes to "gesture" topic. At the end, it waits for the state transition

            @param userdata: used to pass data between states
            @type userdata: list
        """

        # function called when exiting from the node, it can be blacking
        self.arrived = 0
        self.transition = 0
        self.count = 0
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        rospy.loginfo('Executing state PLAY')

        pub_target_pos.publish(person_pos)

        while self.arrived == 0:
            pass

        rospy.loginfo("Robot arrived in person position (%d,%d)",person_pos.x,person_pos.y)

        self.arrived = 0
        sub_gesture = rospy.Subscriber("gesture", Point, self.getGesture)

        while self.transition == 0:
            pass
        sub_gesture.unregister()
        return 'someTimes'

# define state Sleep
class sleep(smach.State):

    """
        A class used to represent the SLEEP behaviour
        of the robot

        Attributes
        --------
        @param arrived: Advise if the robot reaches the position
        @type arrived: int

        Methods
        -----
        getFeedback(data)
            Callback method that received the "arrived" message,waits for a random number of
            seconds and set the attribute arrived to 1
        execute()
            It publishes the home position and, after the robot reaches the position, it
            changes the state to NORMAL
    """

    def __init__(self):

        """
            Constrcutor. It inizializes the attribute and subscribe to "feedback" topic
        """

        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])
        self.arrived = 0
        rospy.Subscriber("feedback", String, self.getFeedback)

    def getFeedback(self,data):

        """
            Callback method that received the "arrived" message,waits for a random number of
            seconds and set the attribute arrived to 1

            @param data: feedback message
            @type data: str
        """

        time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
        self.arrived = 1
        
    def execute(self, userdata):

        """
            It publishes the home position and, after the robot reaches the position, it
            changes the state to NORMAL

            @param userdata: used to pass data between states
            @type userdata: list
        """

        # function called when exiting from the node, it can be blacking
        self.arrived = 0
        rospy.loginfo('Executing state SLEEP')

        pub_target_pos.publish(home_pos)
        
        while self.arrived == 0:
            pass
        rospy.loginfo("Robot arrived in home (%d,%d)",home_pos.x,home_pos.y)
        return 'wakeUp'
    

# define state Normal
class normal(smach.State):

    """
        A class used to represent the NORMAL behaviour
        of the robot

        Attributes
        --------
        @param arrived: Advise if the robot reaches the position
        @type arrived: int
        @param play: Advise if command "play" is arrived
        @type play: int

        Methods
        -----
        getFeedback(data)
            Callback method that received the "arrived" message and set the attribute
            arrived to 1
        getCommand(data)
            Callback method that sets the attribute play to 1
        execute()
            It publishes a random position and, after the robot reaches the position, it
            checks if it necessary to change state (PLAY) or not
    """

    def __init__(self):

        """
            Constrcutor. It inizializes the attributes and subscribe to "feedback" and
            "command" topics
        """

        smach.State.__init__(self,outcomes=['play','someTimes'])
        self.arrived = 0
        self.play = 0
        rospy.Subscriber("command", String, self.getCommand)
        rospy.Subscriber("feedback", String, self.getFeedback)

    def getFeedback(self,data):

        """
            Callback method that received the "arrived" message and set the attribute
            arrived to 1

            @param data: feedback message
            @type data: str
        """

        self.arrived = 1

    def getCommand(self,data):

        """
            Callback method that received the "play" command and set the attribute
            play to 1

            @param data: command message
            @type data: str
        """

        self.play = 1

    def execute(self, userdata):

        """
            It publishes a random position and, after the robot reaches the position, it
            checks if it necessary to change state (PLAY) or not

            @param userdata: used to pass data between states
            @type userdata: list
        """

        self.arrived = 0
        self.play = 0

        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)

        for count in range(0,count_value):
            position = Point()
            position.x = random.randint(1,map_x)
            position.y = random.randint(1,map_y)
            pub_target_pos.publish(position)

            while self.arrived == 0:
                pass
            rospy.loginfo("Robot arrived in (%d,%d)",position.x,position.y)
            if self.play == 1:
                rospy.loginfo("User says PLAY")
                return 'play'
            else:
                self.arrived = 0

        return 'someTimes'

        
def main():

    """
        Main function that initializes the node and the FSM.
        After that it starts the node and the FSM
    """

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