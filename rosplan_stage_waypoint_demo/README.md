# Task-Aware Waypoint Sampling

This package contains:

- A launch file that runs the demo in [Stage](http://wiki.ros.org/stage).
- PDDL files for the inspection mission.
- Scripts to control the mission, including waypoint generation, sampling, task planning, and execution.
- A configuration and script for the ROSPlan sensing interface.

### Demo Description

The demo has been tested with ROS melodic. The demo is an inspection mission that illustrates *Task-Aware Waypoint Sampling*. The demo should do the following:

1.  The simulation, rviz visualisation, and ROSPlan nodes are launched.
2. A goal is posted to deliver an order, which requires making several inspections.
3. A probabilistic roadmap is generated, **N** waypoints are sampled as PDDL objects.
4. The planner is called to generate a plan.
5. If the planner proved the problem unsolvable, **N** is increased and the waypoints are resampled (from 3).
6. If a planner timed out, then **N** is decreased and the waypoints are resampled (from 3).
7. Once a plan is produced, the demo is complete.

See image below showing the demo and rviz visualisation.

<center><img alt="demo screenshot" src="rosplan_waypoint_demo.png" width="50%"></center>

### Installation

Install all the required packages in your catkin workspace:
```
cd ~/ros_ws/src
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone https://github.com/KCL-Planning/rosplan.git
git clone https://github.com/KCL-Planning/rosplan_demos.git
git clone https://github.com/sarah-keren/rob-is.git
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
Begin the simulation, rviz visualisation, and ROSPlan nodes using `lt13_sampling.launch`:
```
roslaunch rosplan_stage_waypoint_demo lt13_sampling.launch
```
The launch file has the following arguments:
  - *approach*: sets the approach to waypoints generation; (0: task-aware waypoint sampling, 1: single dense PRM, 2: fixed waypoint generation). See the next section for more information on each approach.
  - *max_sample_size*: the maximum number of waypoints that will be sampled using the task-aware waypoint sampling approach before returning failure. The default is 50.
  - *max_prm_size*: the size of the PRM from which to sample waypoints, or the maximum size of the PRM in approach 1. The default is 1000 nodes.
  - *objects_file*: path to the object configuration file that specifies the initial positions of each machine in the world. A number of these configuration files are available in the ROB-IS repository.
  - *prefs_file*: optional path to the hidden cost configuration file that specifies some additional preferences over where the robot is allowed to perform inspections. A number of these configuration files are available in the ROB-IS repository.
  - *initial_state*: the initial state file that specifies the goal for the problem, and corresponds to the objects file. These are also available in the ROB-IS repository.

### Sampling Approaches

- **Task-Aware Waypoint Sampling**  
- **Single Dense PRM**  
- **Fixed Waypoint Generation**  
