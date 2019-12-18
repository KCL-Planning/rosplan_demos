# rosplan_demos

[![Build Status](https://travis-ci.com/KCL-Planning/rosplan_demos.svg?branch=master)](https://travis-ci.com/KCL-Planning/rosplan_demos)

Each demo package contains a README with description and instructions for both installation and running.

### Stage Demos
- **Stage Demo** [*rosplan_stage_demo*](tree/master/rosplan_stage_demo) (melodic)  
This demo uses the STAGE simulation, spawning a single robot in simulation. This demo contains the base maps and launches upon which the other stage demos are built.
- **Task-Aware Waypoint Sampling** [*rosplan_waypoint_demo*](tree/master/rosplan_waypoint_demo) (melodic)  
This demonstrates [ROB-IS](https://github.com/sarah-keren/ROB-IS) and builds upon the STAGE demo. The robot is required to complete inspection missions and uses the ROB-IS package for task-aware waypoint sampling.

### Gazebo Demos
- **Turtlebot2 Exploration** [*rosplan_turtlebot2_demo*](tree/master/rosplan_turtlebot2_demo) (kinetic)  
This demo is a simple exploration mission. The robot visits randomly generated waypoints around a map.
- **Turtlebot2 Exploration** [*rosplan_turtlebot3_demo*](tree/master/rosplan_turtlebot3_demo) (melodic)  
This demo is a simple exploration mission. The robot visits randomly generated waypoints around a map.
