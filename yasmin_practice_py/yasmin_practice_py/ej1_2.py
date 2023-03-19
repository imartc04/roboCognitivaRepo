#!/usr/bin/python3

import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub



"""
Explanation of this node

The ros2 node create a yasmin state machine that will be executed uintil 
its exit outcome is reached

The state machine defines 2 states,FOO(of type FooState) and BAR,(of tye BarState) and has one 1 outcome called outcome4

The machine is jumping between the 2 states 3 times until it reaches the state machinie outcome4 through
 the state FOO and the node execution ends

State BAR always returns to state FOO
Satate FOO goes to state BAR 3 times and then goes to the state machine output

"""

# define state Foo
class FooState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    
    def execute(self, blackboard):
        print("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = "Counter: " + str(self.counter)
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["outcome3"])

    def execute(self, blackboard):
        print("Executing state BAR")
        time.sleep(3)

        print(blackboard.foo_str)
        return "outcome3"


class DemoNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["outcome4"])

        # add states
        sm.add_state("FOO", FooState(),
                     transitions={"outcome1": "BAR",
                                  "outcome2": "outcome4"})
        sm.add_state("BAR", BarState(),
                     transitions={"outcome3": "FOO"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        # execute
        outcome = sm()
        print(outcome)


# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
