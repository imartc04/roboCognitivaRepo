

#### How to use the package ####

Set turtlebot env var in terminal:
export TURTLEBOT3_MODEL=waffle

Call laucher in the same terminal: 
roslaunch simple_navigation_ros marsiTurtlebotNavigation.launch


Set turtlebot env var in another terminal:
export TURTLEBOT3_MODEL=waffle

Call services to move of cancel goals: 

Cancel goals : rosservice call /marsi_nav_cancel_goals "{}" 
Move throught waypoints sequential : rosservice call /marsi_nav_move "mode: 0"
Move throught waypoints in random sequence: rosservice call /marsi_nav_move "mode: 1"
Stay in the wait input state : rosservice call /marsi_nav_move "mode: 2"


Before cancelling goals make call to rosservice call /marsi_nav_move "mode: 2" to avoid entering infinitelly in the nav state

