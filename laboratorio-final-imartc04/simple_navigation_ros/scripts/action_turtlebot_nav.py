#!/usr/bin/python3
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from simple_navigation_ros.srv import moveSrv,setPointMapSrv, setInitPoseSrv
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import threading
import smach
import smach_ros
import copy

import yaml
import random
import time
import os
from time import sleep




def getWayPointsFromFile(f_path):
    l_points = {}

    if f_path != "":
        with open(f_path) as file:
            l_points = yaml.load(file, Loader=yaml.FullLoader)
            rospy.loginfo(l_points)

        rospy.loginfo("Points loaded succesfully from %s file", f_path)
    else:
        rospy.loginfo("There is no way points file. Set it through service")
    
    return l_points


        

class CSWaitInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'], 
            output_keys=["moveMode"])

        #Service to order the robot to move
        self.svMove = rospy.Service('marsi_nav_move', moveSrv, self.moveCallback)

        #Servoce to set initial pose coordinates
        self.svInitPose = rospy.Service('marsi_nav_set_initPose', setInitPoseSrv , self.setInitPoseCallback)

        #Publisher
        self.initPosePub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10) 

        self.moveMode = 2
        self.mutexmoveMode = threading.Lock()

        # self.allowConfig = True
        # self.mutexAllowConfig = threading.Lock()

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



    def setInitPoseCallback(self, f_req):
    
        if self.getAllowConfig():

            l_initPoints = []

            #Open initial pose file 
            with open(f_req.initPoseFile) as file:
                l_initPoints = yaml.load(file, Loader=yaml.FullLoader)
            
            rospy.loginfo("Initial points %s", l_initPoints)
            l_initPoint = l_initPoints[f_req.initPosId]

            #Set pose in rviz
            l_msg = PoseWithCovarianceStamped()
            l_msg.header.stamp = rospy.Time.now()
            l_msg.header.frame_id = "/map"

            print("Init point ", l_initPoint)

            l_msg.pose.pose.position.x = l_initPoint[0]
            l_msg.pose.pose.position.y = l_initPoint[1]

            l_msg.pose.pose.orientation.x = l_initPoint[2]
            l_msg.pose.pose.orientation.y = l_initPoint[3]
            l_msg.pose.pose.orientation.z = l_initPoint[4]
            l_msg.pose.pose.orientation.w = l_initPoint[5]

            #Obtain topic publisher
            rospy.loginfo("Setting initial pose to %s,%s", l_initPoint[0], l_initPoint[1])

            self.initPosePub.publish(l_msg)

        else:
            print("Cannot change init pose while the robot is moving")
            print("Stop it first")

        #To avoid error in console
        return []


    def moveCallback(self, f_mode):
        print("New move mode ", f_mode.mode)
        self.setMoveMode(f_mode.mode)

        #To avoid error in console
        return []
        
   

    def execute(self, userdata):

        #Wait until move order
        l_mvMode = self.getMoveMode()
        while l_mvMode != 1 and l_mvMode != 0 :
            sleep(0.1)
            l_mvMode = self.getMoveMode()
        
        userdata.moveMode = l_mvMode

        return 'outcome1'



class CSNav(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'], 
            input_keys=["moveMode"],
        )

        self.actionClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        print("move_base action client waiting for server...")
        self.actionClient.wait_for_server()

        #Service to cancel the robot route
        self.cancelSrv = rospy.Service('marsi_nav_cancel_goals', Empty , self.cancelGoalsCallback)

    
        self.goalsCancelled = False
        self.mutexGoalsCancel = threading.Lock()

        #Variable to store 
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Construct the path to the file relative to the script
        file_path = os.path.join(script_dir, './../rsc/housePoints.yaml')
        self.pointsMapPath = str(file_path)


    def setWayPointsCallback(self, f_map):

        if self.getAllowConfig():
            self.setMapPath(f_map.pointMapPath)
            rospy.loginfo("New points map is in path %s ", f_map)
        else:
            rospy.loginfo("Stop the robot first to allow config")

        #To avoid error in console
        return []

    def cancelGoalsCallback(self, f_empty):
        self.setGoalsCancelled(True)
        self.actionClient.cancel_all_goals()
        sleep(0.2)
        rospy.loginfo("Cancelling all goals")

        #To avoid error in console
        return []

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


    def execute(self, userdata):

        #Clear all possible goals before begin
        self.actionClient.cancel_all_goals()
        time.sleep(0.2)

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        l_points = getWayPointsFromFile(self.pointsMapPath)

        if len(l_points)>0:

            l_loopIds = [*range(len(l_points))]

            #Set random order if needed
            if userdata.moveMode == 1:
                random.shuffle(l_loopIds) 


            #Go over all the points 
            l_ctr = 0
            for i in l_loopIds:

                if self.getGoalsCancelled():
                    rospy.loginfo("Goals have been canceled")
                    break

                else:
                    rospy.loginfo("Going point number  %s", l_ctr)
                    l_point = l_points[i]

                    goal.target_pose.header.stamp = rospy.Time.now()

                    # Set position 
                    goal.target_pose.pose.position.x = l_point[0]
                    goal.target_pose.pose.position.y = l_point[1]

                    #Set orientation
                    goal.target_pose.pose.orientation.x = l_point[2]
                    goal.target_pose.pose.orientation.y = l_point[3]
                    goal.target_pose.pose.orientation.z = l_point[4]
                    goal.target_pose.pose.orientation.w = l_point[5]

                    # Sends the goal to the action server.
                    self.actionClient.send_goal(goal)

                    # Waits for the server to finish performing the action.
                    wait = self.actionClient.wait_for_result()

                    rospy.loginfo("WayPoint reached")

                    l_ctr = l_ctr + 1

            rospy.loginfo("Route through way points done")

        else:
            rospy.loginfo("No points to move through!!! Set waypoints file with service")


        return "outcome1"



def generateSM():

    sm = smach.StateMachine(outcomes=['outcome5'])
    sm.userdata.moveMode = 0
    with sm:
        smach.StateMachine.add("WaitInput", CSWaitInput(),
            transitions={"outcome1":"Nav"},
            remapping={"moveMode":"moveMode"}
            )

        smach.StateMachine.add("Nav", CSNav(),
            transitions={"outcome1":"WaitInput"},
            remapping={ "moveMode":"moveMode"}
            )

    return sm



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('marsi_nav')
        
        sm = generateSM()

        sis = smach_ros.IntrospectionServer('marsi_nav', sm, '/SM_ROOT')
        sis.start()

        sm.execute()


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")