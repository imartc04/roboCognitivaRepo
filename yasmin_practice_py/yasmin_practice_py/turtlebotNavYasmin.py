#!/usr/bin/python3
# license removed for brevity

import rclpy

# Brings in the SimpleActionClient
#import actionlib
# Brings in the .action file and messages used by the move base action
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from yasmin_prac_if.srv import Move
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
#from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

import logging

from simple_node import Node
import yasmin
from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

import threading
import copy

import yaml
import random
import time
import os
from time import sleep




def getWayPointsFromFile(f_path):

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("getWayPoint")

    l_points = {}

    if f_path != "":
        with open(f_path) as file:
            l_points = yaml.load(file, Loader=yaml.FullLoader)
            #rclpy.logging.get_logger("getWayPoint").info(l_points)

        rclpy.logging.get_logger("getWayPoint").info(f"Points loaded succesfully from {f_path} file")
    else:
        rclpy.logging.get_logger("getWayPoint").info("There is no way points file. Set it through service")
    
    return l_points

        

class CSWaitInput(yasmin.State):

    def __init__(self, f_node):
        super().__init__(["outcome1"]) 

        #Service to order the robot to move
        self.svMove = f_node.create_service(Move,'marsi_nav_move', self.moveCallback)
        self.node = f_node
        self.moveMode = 2
        self.mutexmoveMode = threading.Lock()

    def setMoveMode(self, f_value):
        self.mutexmoveMode.acquire()
        self.moveMode = f_value
        self.mutexmoveMode.release()

    def getMoveMode(self):
        l_ret = None
        self.mutexmoveMode.acquire()
        l_ret = copy.deepcopy(self.moveMode)
        self.mutexmoveMode.release()
        return l_ret


    def moveCallback(self, f_mode, response):
        print("New move mode ", f_mode.mode)
        self.setMoveMode(f_mode.mode)

        #To avoid error in console
        #return []
        return response
   

    def execute(self, f_blackboard):

        #Wait until move order
        #rclpy.spin_once(self.node, timeout_sec=0.2) #Give oportunity to callbacks to reach data
        l_mvMode = self.getMoveMode()
        l_ctr = 0
        while l_mvMode != 1 and l_mvMode != 0 :
            # Spin the node with a short timeout
            sleep(0.2)
            #rclpy.spin_once(self.node, timeout_sec=0.2)
            l_mvMode = self.getMoveMode()
            l_ctr +=1
            if l_ctr==15:
                l_ctr =0
                print("Waiting for move order ...")
        
        f_blackboard.moveMode = l_mvMode

        return "outcome1"



class CSNav(yasmin.State):

    def __init__(self, f_node):
        super().__init__(["outcome1"])

        self.node = f_node
        self.navigator = BasicNavigator()

        

        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -3.65
        initial_pose.pose.position.y = 1.52
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()
        

        # self.actionClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # print("move_base action client waiting for server...")
        # self.actionClient.wait_for_server()

        #Service to cancel the robot route
        self.cancelSrv = f_node.create_service(Empty, 'marsi_nav_cancel_goals', self.cancelGoalsCallback)


        self.goalsCancelled = False
        self.mutexGoalsCancel = threading.Lock()

        #Variable to store 
        package_path = get_package_share_directory("yasmin_practice_py")

        # Get the path to a file within the package
        file_name = "rsc/housePoints.yaml"
        file_path = os.path.join(package_path, file_name)

        # Construct the path to the file relative to the script
        self.pointsMapPath = str(file_path)



        #Publish initialpose to amcl
        pub = f_node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set the position and orientation of the robot
        pose_msg.pose.pose.position.x = -3.65
        pose_msg.pose.pose.position.y = 1.52
        pose_msg.pose.pose.position.z = 0.0

        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.08
        pose_msg.pose.pose.orientation.w = 0.99

        # Set the covariance matrix
        pose_msg.pose.covariance = [0.0] * 36

        pub.publish(pose_msg)
     

    def cancelGoalsCallback(self, f_empty, response):
        self.setGoalsCancelled(True)
        self.navigator.cancelTask()
        sleep(0.2)
        self.node.get_logger().info("Cancelling all goals")

        #To avoid error in console
        return response

    def setGoalsCancelled(self, f_value):
        self.mutexGoalsCancel.acquire()
        self.goalsCancelled = f_value
        self.mutexGoalsCancel.release()

    def getGoalsCancelled(self):
        l_ret = None
        self.mutexGoalsCancel.acquire()
        l_ret = copy.deepcopy(self.goalsCancelled)
        self.mutexGoalsCancel.release()
        return l_ret


    def execute(self, f_blackboard):

        self.node.get_logger().info("Inside navigation ....")
        self.setGoalsCancelled(False)

        #Clear all possible goals before begin
        #elf.actionClient.cancel_all_goals()
        time.sleep(0.2)

        # Creates a new goal with the MoveBaseGoal constructor
        goal = PoseStamped()
        goal.header.frame_id = "map"

        l_points = getWayPointsFromFile(self.pointsMapPath)

        if len(l_points)>0:

            l_loopIds = [*range(len(l_points))]

            #Set random order if needed
            if f_blackboard.moveMode == 1:
                random.shuffle(l_loopIds) 


            #Go over all the points 
            l_ctr = 0
            for i in l_loopIds:

                if self.getGoalsCancelled():
                    self.node.get_logger().info("Goals have been canceled")
                    break

                else:
                    self.node.get_logger().info(f"Going point number  {l_ctr}")
                    l_point = l_points[i]

                    goal.header.stamp = self.navigator.get_clock().now().to_msg()

                    # Set position 
                    goal.pose.position.x = l_point[0]
                    goal.pose.position.y = l_point[1]

                    #Set orientation
                    goal.pose.orientation.x = l_point[2]
                    goal.pose.orientation.y = l_point[3]
                    goal.pose.orientation.z = l_point[4]
                    goal.pose.orientation.w = l_point[5]

                    # Sends the goal to the action server.
                    self.navigator.goToPose(goal)

                    while not self.navigator.isTaskComplete():
                        sleep(0.2)

                    self.node.get_logger().info("WayPoint reached")

                    l_ctr = l_ctr + 1

            self.node.get_logger().info("Route through way points done")

        else:
            self.node.get_logger().info("No points to move through!!! Set waypoints file with service")


        return "outcome1"



def spinNodeThread(f_node, executor):
    print("Spinning on node")
    executor.spin()
    print("After spin")



def main(args=None):
    rclpy.init()

    #Create node 
    node = rclpy.create_node("marsi_nav")


    # Create executors for each thread
    main_executor = rclpy.executors.SingleThreadedExecutor()
    callback_executor = rclpy.executors.SingleThreadedExecutor()
    main_executor.add_node(node)
    callback_executor.add_node(node)


    # create a state machine
    sm = StateMachine(outcomes=["outcome4"])

    # add states
    sm.add_state("WAIT_INPUT", CSWaitInput(node),
                    transitions={"outcome1": "NAV"})
                                                
    sm.add_state("NAV", CSNav(node),
                    transitions={"outcome1": "WAIT_INPUT"})

    # pub
    YasminViewerPub(node, "turtlebot3Nav", sm)


    # Create thread to spin on node and allow it to call on callbacks
    thread = threading.Thread(target=spinNodeThread, args=(node, callback_executor))

    print("Before thread start")

    # Start the thread
    thread.start()


    print("Executing state machine........................")
    # execute
    outcome = sm()
    print(outcome)


    node.destroy_node()
    rclpy.shutdown()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    main()
