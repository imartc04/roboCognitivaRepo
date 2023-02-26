#!/usr/bin/python3

import rospy
import smach
import std_msgs
import smach_ros

# Define the CS_Sub1 state
class CS_Sub1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):

        rospy.loginfo("In state CS_Sub1...")
        # Wait for the goal to be reached
        rospy.sleep(1)
   
        # Return the appropriate outcome
        return 'out1'

# Define the subSM state machine
class subSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['out3'])
        with self:
            # Add the CS_Sub1 state to the subSM state machine
            smach.StateMachine.add('CS_Sub1', CS_Sub1(), transitions={'out1':'out3'})
          

# Define the topSM state machine
class topSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['out4'])
        with self:
            # Add the CS_Top1 state to the topSM state machine
            smach.StateMachine.add('CS_Top1', CS_Top1(), transitions={'out1':'subSM', 'out2':'CS_Top1'})
            # Add the subSM state machine to the topSM state machine
            smach.StateMachine.add('subSM', subSM(), transitions={'out3':'out4'})

# Define the CS_Top1 state
class CS_Top1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1', 'out2'])
        self.message = ""
        rospy.loginfo("Waiting for smAdvance topic in publishiing something")
        self.subscriber = rospy.Subscriber('/smAdvance', std_msgs.msg.String, self.callback)

    def execute(self, userdata):
        # Wait for the smAdvance topic to be published

        rospy.loginfo("In state CS_Top1, waiting for message tod advance ( rostopic pub /smAdvance std_msgs/String 'advance')")
        rospy.sleep(0.5)
        # Return the appropriate outcome based on which topic was received
        if self.message == 'advance':
            return 'out1'
        else:
            return 'out2'

    def callback(self, message):
        # Store the message for later use
        self.message = message.data

if __name__ == '__main__':
    rospy.init_node('smach_example')

    # Create the topSM state machine
    sm = topSM()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()

