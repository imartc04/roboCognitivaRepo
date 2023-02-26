#!/usr/bin python3

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import random

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['out1','out2'],
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'out1'
        else:
            return 'out2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['out1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'out1'
        

#Define new state that stays in itself 70% of the time
class CSt3(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['out1', 'out2'],
                             )
        
    def execute(self, userdata):
        
        if random() >= 0.7:
            return 'out1'
        else:
            return 'out2'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'out1':'BAR', 
                                            'out2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'out1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})

        smach.StateMachine.add('CSt3', CSt3(), 
                               transitions={'out1':'BAR',
                                            'out2':'CSt3'
                               }
        )


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
