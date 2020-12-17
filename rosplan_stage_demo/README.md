# Stage Demo

This package contains the base maps and launches upon which the other stage demos are built. Please look to the other demos in this repository for demonstrations of different planning and execution tools with ROSPlan.

The package contains:
- An example launch file for a robot simulation in [Stage](http://wiki.ros.org/stage).
- Stage world files, maps, and configuration files.

#### Demo Description

The demo has been tested with ROS melodic.

The demo launches a Stage simulation with the following components:
- *stageros* simulation node.
- *map_server* which offers map data (http://wiki.ros.org/map_server)
- *turtlebot3_bringup* scripts for starting the robot (http://wiki.ros.org/turtlebot3_bringup)
- *move_base* (http://wiki.ros.org/move_base) and *amcl* (http://wiki.ros.org/amcl) for navigation.
- *rviz* for visualisation.

![demo screenshot](stage_demo.png)

### Installation

Fetch the required dependencies:
```
sudo apt install ros-${ROS_DISTRO}-turtlebot3-navigation ros-${ROS_DISTRO}-move-base-msgs ros-${ROS_DISTRO}-dwa-local-planner
```

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
Begin the simulation and rviz visualisation:
```
roslaunch rosplan_stage_demo empty_stage.launch
```
