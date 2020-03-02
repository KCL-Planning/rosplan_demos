# Exploration demo with turtlebot 3 and Gazebo simulator (melodic)

### Demo brief description

The turtlebot demo is a simple exploration mission. The robot will visit randomly generated waypoints around a map.

See image below showing gazebo and rviz visualisation.

<img src="doc/turtle_demo_gazebo.jpg" width="50%" height="50%"><img src="doc/turtle_demo.png" width="50%" height="50%">

### Turtlebot PDDL domain

The PDDL domain definition of this demo can be accessed by doing:

```
roscat rosplan_turtlebot3_demo domain_turtlebot_demo.pddl
rosed rosplan_turtlebot3_demo domain_turtlebot_demo.pddl
```

## Installation instructions

The demo requires that you install turtlebot 3 gazebo simulator and some other ros debian pkgs, this will be done later on using rosdep, for now setup only build dependencies.
```
cd ~/rosplan_ws/src
git clone https://github.com/KCL-Planning/ROSPlan.git
git clone https://github.com/KCL-Planning/rosplan_demos.git
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
```

Compile the code:
```
catkin build
```

Source the ROSPlan workspace and install runtime dependencies:
```
source ~/rosplan_ws/devel/setup.bash
rosdep install rosplan_turtlebot3_demo
```

## Run instructions

NOTE: The demo requires 2 terminals.

*1.* In the first terminal, export the desired turtlebot 3 configuration, available options include: burger, waffle, waffle_pi, e.g.:
```
export TURTLEBOT3_MODEL=waffle
```

*2.* In the same terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `turtlebot.launch` from the `rosplan_demos` package:
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

## Troubleshoot

### Rotating behavior on the robot while navigating between waypoints.

NOTE: You might experience a rotating behavior on the robot (overshooting the local trajectory). This is a known issue, however it should make progress towards the goal and eventually reach the waypoints. This could be fixed by tunning the parameters for the mobile base, however is out of the scope of this demo.

### Gazebo Turtlebot 3, invalid lookupTransform tf2 error (melodic)

If you experience trouble with melodic regarding a tf2 invalid lookupTransform, as a workaround you can clone it from source:
```
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```

### Fix small bug in gazebo installation (if needed)

If you get this error in terminal while launching Gazebo turtlebot 3 simulation:

```
[Err] [REST.cc:205] Error in REST request
```

There is a fix in [here](https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo)

Just change ~/.ignition/fuel/config.yaml as following.

Look for the line that says:

    url: https://api.ignitionfuel.org

replace it with:

    url: https://api.ignitionrobotics.org
