
#!/usr/bin/env python3

import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String
import threading

g_msgRec = False
g_mutex = threading.Lock()

# define state Foo
class FooState(State):
    def __init__(self):
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def msgReceived(self):
        global g_msgRec
        global g_mutex

        l_ret = False
        g_mutex.acquire()
        l_ret = g_msgRec
        g_mutex.release()

        return l_ret

    def execute(self, blackboard):
        print("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = "Counter: " + str(self.counter)
            return "outcome1"
        else:
            
            while( not self.msgReceived()):
                time.sleep(2)
                print("Node Foo waiting for message in demoSMTopic to continue ...\n", "Example : ros2 topic pub /demoSMTopic std_msgs/msg/String " '"{data: "hello"}" ' )

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

        #Suscribe to topic
        self.create_subscription(
            String, "demoSMTopic", self.topCallback, 10)

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

    def topCallback(self, msg):
        global g_msgRec
        global g_mutex
        print("****** Message received in topic")
        g_mutex.acquire()
        g_msgRec = True
        g_mutex.release()


# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)

    node = DemoNode()

    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()






