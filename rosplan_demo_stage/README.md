# rosplan_demo_stage

This package contains:

- launch files for different robot simulations in with [Stage](http://wiki.ros.org/stage).
- Stage world files, maps, and configuration files.

#### Demo brief description

The demo has been tested with ROS melodic. The turtlebot demo is a simple exploration mission. The robot will visit randomly generated waypoints around a map.

See image below showing the demo and rviz visualisation.

![demo screenshot](rosplan_demo_stage.png?raw=true)

### Turtlebot PDDL domain

The PDDL domain definition can be found under `rosplan_demo_stage` package -> `pddl/domain_turtlebot_demo.pddl`:

```
roscd rosplan_demo_stage/pddl
cat domain_turtlebot_demo.pddl
```

### Installation instructions

The demo requires that you install ros_stage, you can do so by executing the following commands:

(melodic)
```
sudo apt install ros-melodic-turtlebot3-gazebo ros-${ROS_DISTRO}-turtlebot3-navigation ros-${ROS_DISTRO}-move-base-msgs
```

Install further dependencies from source in your catkin workspace:
```
cd ~/ros_ws/src
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan_demos.git
```

It might be the case that you experience troubles in melodic with tf2 invalid lookupTransform, as a workaround you can clone it from source
```
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
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

*1.* Export the desired turtlebot 3 configuration, available options include: burger, waffle, waffle_pi, e.g.:
```
export TURTLEBOT3_MODEL=waffle
```

*2.* In a first terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `turtlebot.launch` from the `rosplan_demos` package:
```
roslaunch rosplan_turtlebot3_demo turtlebot.launch
```

*3.* In a second terminal run `turtlebot_explore.bash`, a script which:
- adds to the knowledge base the PDDL objects and facts which comprise the initial state.
- adds the goals to the knowledge base.
- calls the ROSPlan services which generate a plan and dispatch it.
```
rosrun rosplan_turtlebot3_demo turtlebot_explore.bash
```

*4.* Alternatively instead of step 3, you can test your own custom waypoint from yaml file:
```
rosed rosplan_turtlebot3_demo waypoints.yaml
rosrun rosplan_turtlebot3_demo turtlebot_explore_wp_from_file.bash
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

