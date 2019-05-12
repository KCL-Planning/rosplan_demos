# rosplan_demos

[![Build Status](https://travis-ci.com/KCL-Planning/rosplan_demos.svg?branch=master)](https://travis-ci.com/KCL-Planning/rosplan_demos)

This repository contains:

- the main rosplan turtlebot exploration demo (previously: rosplan_demos, now ->pkg: rosplan_turtlebot2_demo)
- rosplan interfaces (move base and mapping) for the above mentioned demo (meta-pkg: rosplan_demos_interfaces)
- general demo files (pkg: rosplan_demos), kept here for backwards compatibility purposes

### Running a demo with the turtlebot 2, using Gazebo simulator (kinetic only)

NOTE for ```melodic``` users: Because turtlebot 2 simulation debian packages have not been released yet into melodic (last udpate 15.03.2019), this demo is now guaranteed to work only for kinetic, read more about it in this [issue](https://github.com/KCL-Planning/ROSPlan/issues/180).

If you wish to test in ```kinetic```, feel free to continue reading:

### Demo brief description

The turtlebot demo is a simple exploration mission.

The turtlebot will visit randomly generated waypoints around a map. See image below showing rviz visualisation.

![](rosplan_turtlebot2_demo/doc/turtle_demo.png?raw=true)

### Turtlebot PDDL domain

The PDDL domain file for this demo can be found under `rosplan_turtlebot2_demo` package, as `common/domain_turtlebot_demo.pddl`:

```
roscd rosplan_turtlebot2_demo/common
cat domain_turtlebot_demo.pddl
```

### Installation instructions

To run the demo first follow the installation instructions and quick-start guide for the [Turtlebot Gazebo simulator](http://wiki.ros.org/turtlebot_gazebo) and [Turtlebot Simulator](http://wiki.ros.org/turtlebot_simulator).

Install debian dependencies (simulator, move base and mongoDB)
```
sudo apt install ros-kinetic-turtlebot-gazebo ros-kinetic-move-base-msgs ros-kinetic-mongodb-store
```

Install further dependencies from source in your catkin workspace:
```
cd ~/ros_ws/src
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan_demos.git
```

Compile the code:
```
catkin build
```

Then source the ROSPlan workspace in two terminals.
```
source ~/ros_ws/devel/setup.bash
```

### Run instructions

*1.* In a first terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `turtlebot.launch` from the `rosplan_demos` package:
```
roslaunch rosplan_turtlebot2_demo turtlebot.launch
```

*2.* In a second terminal run `turtlebot_explore.bash`, a script which:
- adds to the knowledge base the PDDL objects and facts which comprise the initial state;
- adds the goals to the knowledge base; and
- calls the ROSPlan services which generate a plan and dispatch it.
```
rosrun rosplan_turtlebot2_demo turtlebot_explore.bash
```

You should see the following output from the script:
```
waypoints: ['wp0', 'wp1', 'wp2', 'wp3', 'wp4', 'wp5']
Adding initial state and goals to knowledge base.
success: True
success: True
Calling problem generator.
Calling planner interface.
Calling plan parser.
Calling plan dispatcher.
Finished!
```

The turtlebot will move around the waypoints, exploring the environment. You should see output in the first terminal, something like:
```
...
KCL: (/rosplan_problem_interface) (problem.pddl) Generating problem file.
KCL: (/rosplan_problem_interface) (problem.pddl) The problem was generated.
KCL: (/rosplan_planner_interface) Problem received.
KCL: (/rosplan_planner_interface) (problem.pddl) Writing problem to file.
KCL: (/rosplan_planner_interface) (problem.pddl) Running: timeout 10 /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_planning_system/common/bin/popf /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/domain_turtlebot_demo.pddl /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/problem.pddl > /home/michael/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/plan.pddl
KCL: (/rosplan_planner_interface) (problem.pddl) Planning complete
KCL: (/rosplan_planner_interface) (problem.pddl) Plan was solved.
KCL: (/rosplan_parsing_interface) Planner output received.
KCL: (/rosplan_parsing_interface) Parsing planner output.
KCL: (/rosplan_plan_dispatcher) Plan received.
KCL: (/rosplan_plan_dispatcher) Dispatching plan.
KCL: (/rosplan_plan_dispatcher) Dispatching action [0, goto_waypoint, 0.804106, 10.000000]
KCL: (/rosplan_plan_dispatcher) Feedback received [0, action enabled]
...
```
NOTE: You might experience a rotating behavior on the robot (overshooting the local trajectory), this is a known issue, however it should make progress towards the goal and eventually reach the waypoints.

