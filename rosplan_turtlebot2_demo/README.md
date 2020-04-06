# Exploration demo with turtlebot 2 and Gazebo simulator

This demo will work "out of the box" using mostly debian pkgs under ubuntu 16.04 + kinetic.

For ubuntu 18.04 users we recommend to visit the [turtlebot 3 demo](https://github.com/KCL-Planning/rosplan_demos/tree/master/rosplan_turtlebot3_demo),

however if you really wish to try turtlebot 2 gazebo demo under ubuntu 18.04 + melodic we present instructions on

how to do this by cloning the code from source.

## Demo brief description

The turtlebot demo is a simple exploration mission. The robot will visit randomly generated waypoints around a map.

See image below showing rviz visualisation.

![](doc/turtle_demo.png?raw=true)

## Turtlebot PDDL domain

The PDDL domain definition can be found under `rosplan_turtlebot2_demo` package -> `common/domain_turtlebot_demo.pddl`:

```
roscd rosplan_turtlebot2_demo/common
cat domain_turtlebot_demo.pddl
```

## Installation instructions (ubuntu 16.04 + kinetic)

The demo requires that you install turtlebot gazebo simulator, you can do so by executing the following commands:

turtlebot 2 (kinetic only)
```
sudo apt install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-navigation ros-kinetic-move-base-msgs
```

Install [ROSPlan](https://github.com/kcl-planning/ROSPlan) and further dependencies from source in your catkin workspace:
```
cd ~/rosplan_ws/src
git clone https://github.com/KCL-Planning/ROSPlan.git
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan_demos.git
```

Compile the code:
```
catkin build
```

Then source the ROSPlan workspace in two terminals.
```
source ~/rosplan_ws/devel/setup.bash
```

## New : Installation instructions (ubuntu 18.04 + melodic from source)

Install debian pkgs:

```
sudo apt install ros-melodic-kobuki-msgs ros-melodic-ecl-exceptions ros-melodic-ecl-threads
ros-melodic-ecl-geometry ros-melodic-kobuki-dock-drive ros-melodic-yocs-controllers
ros-melodic-kobuki-driver ros-melodic-ecl-streams ros-melodic-yocs-velocity-smoother
ros-melodic-depthimage-to-laserscan ros-melodic-yocs-cmd-vel-mux ros-melodic-gazebo-plugins pyqt5-dev-tools
```

clone turtlebot 2 repos:

```
git clone --branch melodic https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/kobuki_desktop.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
```

Compile the code:
```
catkin build
```

Then source the ROSPlan workspace in two terminals.
```
source ~/rosplan_ws/devel/setup.bash
```

## Run instructions

*1.* In a first terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `turtlebot.launch` from the `rosplan_demos` package:
```
roslaunch rosplan_turtlebot2_demo turtlebot.launch
```

*2.* In a second terminal run `turtlebot_explore.bash`, a script which:
- adds to the knowledge base the PDDL objects and facts which comprise the initial state.
- adds the goals to the knowledge base.
- calls the ROSPlan services which generate a plan and dispatch it.
```
rosrun rosplan_turtlebot2_demo turtlebot_explore.bash
```

*3.* Alternatively instead of step 2, you can test your own custom waypoint from yaml file:
```
rosed rosplan_turtlebot2_demo waypoints.yaml
rosrun rosplan_turtlebot2_demo turtlebot_explore_wp_from_file.bash
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
KCL: (/rosplan_planner_interface) (problem.pddl) Running: timeout 10 /home/user/ros_indigo/turtlebot/src/rosplan/rosplan_planning_system/common/bin/popf /home/user/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/domain_turtlebot_demo.pddl /home/user/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/problem.pddl > /home/user/ros_indigo/turtlebot/src/rosplan/rosplan_demos/common/plan.pddl
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
