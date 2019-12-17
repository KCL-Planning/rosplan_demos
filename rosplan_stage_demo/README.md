# rosplan_demo_stage

This package contains:

- launch files for different robot simulations in with [Stage](http://wiki.ros.org/stage).
- Stage world files, maps, and configuration files.

#### Demo brief description

The demo has been tested with ROS melodic. The demo is a simple exploration mission and will follow these steps:
- The simulation, rviz visualisation, and ROSPlan nodes are launched.
- A probabilistic roadmap is generated and loaded into the ROSPlan knowledge base as PDDL objects.
- A goal is posted to visit these randomly generated waypoints.
- The planner is called to generate a plan.
- The plan is executed (and the robot moves).

See image below showing the demo and rviz visualisation.

![demo screenshot](rosplan_demo_stage.png?raw=true)

### Turtlebot PDDL domain

The PDDL domain definition can be found under `rosplan_demo_stage` package -> `pddl/domain_turtlebot_demo.pddl`:

```
roscd rosplan_demo_stage/pddl
cat domain_turtlebot_demo.pddl
```
### Run instructions

*1.* Export the turtlebot 3 configuration, e.g.:
```
export TURTLEBOT3_MODEL=waffle
```

*2.* In a first terminal, begin the simulation, rviz visualisation, and ROSPlan nodes using the `lt13.launch` from the `rosplan_demo_stage` package:
```
roslaunch rosplan_demo_stage lt13.launch
```

*3.* In a second terminal generate the probabilistic roadmap using a service call to the `rosplan_roadmap_server`:
```
rosservice call /rosplan_roadmap_server/create_prm "{nr_waypoints: 75, min_distance: 1.2, casting_distance: 2.4, connecting_distance: 4.8, occupancy_threshold: 50, total_attempts: 100000}"
```
This service call uses the following parameters:
- `nr_waypoints`: The number of waypints to attempt to generate (75).
- `min_distance`: The closest two waypoints are allowed to be (1.2m).
- `casting_distance`: When a waypoint is generated, it will be this distance from its parent (2.4m).
- `connecting_distance`: Edges can be declared in the PDDL problem between any pair of waypoints at least this close (4.8m). However, the launch file disables edges.
- `occupancy_threshold`: A cell in the costmap will be considered collision-free if it is less than this value (90).
- `total_attempts`: The maximum number of attempts to generate a waypoint. This limits the PRM generation time, and might mean that less than 75 waypoints are generated in total (for example, if the min_distance is too high and there is not enough space available).

*4.* In a second terminal run `turtlebot_explore_common.bash`, a script which:
- Adds to the knowledge base the robot PDDL object and the facts which comprise the initial state.
- Adds the goals (to visit all waypoints) the knowledge base.
- Calls the ROSPlan services which generate a plan and dispatch it.
```
rosrun rosplan_turtlebot3_demo turtlebot_explore_common.bash
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
