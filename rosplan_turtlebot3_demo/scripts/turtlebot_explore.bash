#!/bin/bash

# call rosplan mapping interface to generate waypoints based on costmap
echo "Generating waypoints.";
rosservice call /rosplan_roadmap_server/create_prm "{nr_waypoints: 10, min_distance: 0.3, casting_distance: 2.0, connecting_distance: 8.0, occupancy_threshold: 10, total_attempts: 10}";

# call turtlebot_explore_common.bash script
bash `rospack find rosplan_turtlebot2_demo`/scripts/turtlebot_explore_common.bash
