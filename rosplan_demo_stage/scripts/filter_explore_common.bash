#!/bin/bash
sleep 5

rosservice call /rosplan_roadmap_server/create_prm "{nr_waypoints: 1000, min_distance: 0.8, casting_distance: 1.6, connecting_distance: 2.0, occupancy_threshold: 50, total_attempts: 100000}"

rosservice call /roadmap_filter/filter_waypoints

# add robot kenny instance + goals: visit all waypoint instances
echo "Adding initial state and goals to knowledge base.";
param_type="update_type:
- 0";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
param_type="$param_type
- 0";
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'r', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";
for i in $(rosservice call /rosplan_knowledge_base/state/instances "type_name: 'waypoint'" | grep -ohP "wp\d+")
do
param_type="$param_type
- 1"
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: '$i'}
  function_value: 0.0"
done;

rosservice call /rosplan_knowledge_base/update_array "
$param_type
$param"

# NOTE: robot_at(kenny wp0) gets added by the mapping interface

# automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

# make plan (e.g. call popf to create solution)
echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

echo "Finished!";
