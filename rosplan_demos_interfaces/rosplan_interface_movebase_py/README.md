# rosplan_interface_movebase_py

Demo on how to use the [RPActionInterface python interface](https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_planning_system/src/rosplan_planning_system/ActionInterfacePy/RPActionInterface.py) 
with [move base](http://wiki.ros.org/move_base).

We provide a use case with the turtlebot3.

## Test with turtlebot3

Launch required components:

    roslaunch rosplan_interface_movebase_py rosplan_interface_movebase_py_demo.launch

Run ROSPlan coordination script:

    rosrun rosplan_turtlebot3_demo turtlebot_explore.bash

Done. Watch in rviz the turtlebot3 navigating through some waypoints.
