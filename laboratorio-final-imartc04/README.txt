####### Description #######   

This package sends goals to move_base node. The goals are obtained from a .yaml file and can be sent in sequential or 
random order. An order to stop the goal send process can be also given to move_base

####### Services offered ####### 

marsi_nav_set_waypoints : 
    Set the yaml waypoint file. 
    The file has the following format
        id1 : [x,y, orientation_x, orientation_y, orientation_z, orientation_w]
        ...
        idn : [x,y, orientation_x, orientation_y, orientation_z, orientation_w]

    id numbers are Natural numbers
    Orientations are values of each quaternion component

marsi_nav_move : 
    Send goals or stop the process. Possible values passed are 0:Sequential between way points,
    1:Random between way points, 2:Cancel movement

 marsi_nav_set_initPose
    Send initial pose to AMCL


####### Published topics #######

/initialpose
/move_base/goal

####### How to use #######

- Execute launch file : roslaunch simple_navigation_ros marsiTurtlebotNavigation.launch

- Give marsi_nav node way points file : rosservice call /marsi_nav_set_waypoints "pointMapPath: '<path to housePoints.yaml'"

- Order robot to move : rosservice call /marsi_nav_move "mode: 0"



####### Video demos #######

https://drive.google.com/drive/folders/1luuHtvInq9yioKXQ9o_aDTu__nHb3SXM?usp=share_link
