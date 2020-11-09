# rosplan_demos

![Build status](https://github.com/KCL-Planning/rosplan_demos/workflows/build/badge.svg)

Each demo package contains a README with description and instructions for both installation and running.

# Installation

This repository contains demos for [ROSPlan](https://github.com/KCL-Planning/rosplan).
Please refer to the [ROSPlan installation instructions](https://github.com/KCL-Planning/ROSPlan/blob/master/README.md#installation) before proceeding.

Additionally, in order to succesfully compile this repository, please do:

    sudo apt install ros-$ROS_DISTRO-tf2-bullet

For ROS noetic:

    git clone --branch noetic-devel git@github.com:galou/occupancy_grid_utils.git

For ROS melodic:

    git clone git@github.com:clearpathrobotics/occupancy_grid_utils.git

Build

    cd <your catkin workspace>
    catkin build

### Stage Demos

- **Stage Demo** [*rosplan_stage_demo*](https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_demo) (melodic)  
This demo uses the STAGE simulation, spawning a single robot in simulation. This demo contains the base maps and launches upon which the other stage demos are built.
<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_demo/stage_demo.png" width="50%"></p>

- **Stage Exploration** [*rosplan_stage_exploration_demo*](https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_exploration_demo) (melodic)  
This demo uses the STAGE simulation, spawning a single robot to visit one or more waypoints. The demo includes generating a problem, planning, and executing a very simple plan.
<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_exploration_demo/rosplan_exploration_demo.png" width="50%"></p>

- **Task-Aware Waypoint Sampling** [*rosplan_stage_waypoint_demo*](https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_waypoint_demo) (melodic)  
This demonstrates [ROB-IS](https://github.com/sarah-keren/ROB-IS) and builds upon the STAGE demo. The robot is required to complete inspection missions and uses the ROB-IS package for task-aware waypoint sampling.
<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_stage_waypoint_demo/rosplan_waypoint_demo.png" width="25%"></p>

### Gazebo Demos

- **Turtlebot2 Exploration** [*rosplan_turtlebot2_demo*](https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_turtlebot2_demo) (kinetic)  
This demo is a simple exploration mission. The robot visits randomly generated waypoints around a map.
<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_turtlebot2_demo/doc/turtle_demo.png" width="25%"></p>

- **Turtlebot3 Exploration** [*rosplan_turtlebot3_demo*](https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_turtlebot3_demo) (melodic, noetic)
This demo is a simple exploration mission. The robot visits randomly generated waypoints around a map.
<p align="center"><img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_turtlebot3_demo/doc/turtle_demo_gazebo.jpg" width="25%" height="25%">
                  <img src="https://github.com/KCL-Planning/rosplan_demos/blob/master/rosplan_turtlebot3_demo/doc/turtle_demo.png" width="25%" height="25%"></p>
