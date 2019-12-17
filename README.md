# rosplan_demos

[![Build Status](https://travis-ci.com/KCL-Planning/rosplan_demos.svg?branch=master)](https://travis-ci.com/KCL-Planning/rosplan_demos)

This repository contains:

### Demos
- Stage Demo:                   rosplan_demo_stage (melodic)
- Turtlebot2 Exploration:       rosplan_turtlebot2_demo (kinetic)
- Turtlebot3 Exploration:       rosplan_turtlebot3_demo (melodic)
- Task-Aware Waypoint Sampling: rosplan_waypoint_demo

For each demo, the package contains a README with instructions for installation and running.

### Interface Packages
- rosplan move base and mapping interfaces for the above mentioned demos (meta-pkg: rosplan_demos_interfaces)

## Demo Descriptions

The turtlebot demo is a simple exploration mission. The robot will visit randomly generated waypoints around a map.

See image below showing rviz visualisation.

![](rosplan_turtlebot2_demo/doc/turtle_demo.png?raw=true)
