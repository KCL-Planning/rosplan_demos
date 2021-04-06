# Stage Exploration Demo

This package contains:

- A launch files that run robot exploration demos in [Stage](http://wiki.ros.org/stage).
- PDDL files for the exploration.
- A script to control the mission, performing waypoint generation, task planning, and execution.
- A configuration and script for the ROSPlan sensing and action interfaces.

### Demo Description

The demo has been tested with ROS melodic. The demo is a simple exploration in the Stage simulation. The demo should do the following:

1. The simulation, rviz visualisation, RQT visualisation of the plan, and ROSPlan nodes are launched.
2. A probabilistic roadmap is generated, shown as a blue graph in rviz.
3. A goal is posted to visit some random waypoints.
4. The planner is called to generate a plan.
6. Once a plan is produced, the plan is dispatched. The robot(s) will travel through connected waypoints to the goal, and the state of the plan will be shown in the RQT window.
7. Once the goal is achieved, everything is closed.

See image below showing the demo and rviz visualisation.

<center><img alt="demo screenshot" src="rosplan_exploration_demo.png" width="90%"></center>

### Installation

Install all the required packages in your catkin workspace:
```
cd ~/ros_ws/src
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan.git
git clone https://github.com/KCL-Planning/rosplan_demos.git
```
Compile the code:
```
catkin build
```

### Running

-  **Export the Turtlebot3 configuration**  
Also remember to source the workspace.
```
export TURTLEBOT3_MODEL=waffle
source devel/setup.bash
```
- **Launch the demo**  
Begin the simulation, rviz visualisation, and ROSPlan nodes using `lt13_exploration.launch`:
```
roslaunch rosplan_stage_exploration_demo exploration.launch
```
Alternatively, use the multi-robot version:
```
roslaunch rosplan_stage_exploration_demo exploration_multirobot.launch
```

The launch file has the following arguments:
  - *max_prm_size*: the size of the PRM from which to sample waypoints, or the maximum size of the PRM in approach 1. The default is 1000 nodes.
  - *map_file*: the yaml file for map_server to load (should match the stage world file). The default is the simple building of the empty stage demo.
  - *world_file*: the world file for stage to load (should match map_server's yaml file). The default is the simple building of the empty stage demo
