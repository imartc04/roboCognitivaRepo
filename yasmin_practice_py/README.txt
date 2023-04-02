

#### How to use turtlebotNavYasmin.py ####

    #Set the next environment variables 
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models

    Execute nav2 customized launcher: 
    ros2 launch yasmin_practice_py marsiNav2Turtlebot3House.launch.xml

    In another terminal execute yasmin_viewer node

    In another terminal set the environment variables and execute the navigation script:
    ros2 run yasmin_practice_py turtlebotNavYasmin


    To move through way points in sequential order:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 0\

    To move through way points in random order:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 1\

    To stay in wait input position set another number different than 0 and 1:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 10\

    To cancel goals
    ros2 service call /marsi_nav_cancel_goals std_srvs/srv/Empty {}\ 




????????Posible bug en YASMIN ????

Si se pone el nombre de un nodo en con alguna minuscula, por ejemplo abajo Nav en lugar de NAV 
se obtiene el error 

    # add states
    sm.add_state("WAIT_INPUT", CSWaitInput(node),
                    transitions={"outcome1": "NAV"})
                                                
    sm.add_state("NAV", CSNav(node),
                    transitions={"outcome1": "WAIT_INPUT"})

Error 

n main
    outcome = sm()
  File "/sharedMasterRSI/roboticaCognitiva/humble/ros2_ws/install/yasmin/local/lib/python3.10/dist-packages/yasmin/state.py", line 25, in __call__
    outcome = self.execute(blackboard)
  File "/sharedMasterRSI/roboticaCognitiva/humble/ros2_ws/install/yasmin/local/lib/python3.10/dist-packages/yasmin/state_machine.py", line 83, in execute
    raise Exception("Outcome (" + outcome + ") without transition")
Exception: Outcome (Nav) without transition
