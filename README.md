# rosplan_demos

[![Build Status](https://travis-ci.com/KCL-Planning/rosplan_demos.svg?branch=master)](https://travis-ci.com/KCL-Planning/rosplan_demos)

For each demo, the package contains a README with instructions for installation and running.

### Stage Demos
- **Stage Demo** (rosplan_stage_demo) (melodic) This demo uses the STAGE simulation instead of Gazebo. A single robot will be spawned in an empty world.
- **Task-Aware Waypoint Sampling** (rosplan_waypoint_demo) (melodic) This demonstrates the [ROB-IS](https://github.com/sarah-keren/ROB-IS) and builds upon the STAGE demo. The robot is required to complete inspection missions and uses the ROB-IS package for task-aware waypoint sampling.

### Gazebo Demos
- **Turtlebot2 Exploration** (rosplan_turtlebot2_demo) (kinetic) This demo is a simple exploration mission. The robot will be able to visit randomly generated waypoints around a map.
- **Turtlebot2 Exploration** (rosplan_turtlebot3_demo) (melodic) This demo is a simple exploration mission. The robot will be able to visit randomly generated waypoints around a map.
