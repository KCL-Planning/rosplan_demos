#!/bin/bash
rosservice call /rosplan_roadmap_server/create_prm "{nr_waypoints: 1000, min_distance: 0.8, casting_distance: 1.6, connecting_distance: 2.0, occupancy_threshold: 50, total_attempts: 100000}"

rosservice call /waypoint_sampler/sample_waypoints

# automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
echo "Calling problem generator.";
# rosservice call /rosplan_problem_interface/problem_generation_server;

# make plan (e.g. call popf to create solution)
echo "Calling planner interface.";
# rosservice call /rosplan_planner_interface/planning_server;

echo "Finished!";
