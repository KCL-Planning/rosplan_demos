#!/bin/bash

# call rosplan mapping interface to generate waypoints based on costmap
echo "Generating waypoints.";
rosservice call /kcl_rosplan/rosplan_roadmap_server/create_prm "{nr_waypoints: 10, min_distance: 0.3, casting_distance: 2.0, connecting_distance: 8.0, occupancy_threshold: 10, total_attempts: 10}";

# add the fact that robot kenny exists (object instace robot kenny) + goals: visit waypoints, e.g. (visited wp#)
echo "Adding initial state and goals to knowledge base.";
param_type="update_type:
- 0";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
for i in $(seq 1 $(( $(rosservice call /rosplan_knowledge_base/state/instances "type_name: 'waypoint'" | sed 's/wp/\n/g' | wc -l) - 2)) )
do
param_type="$param_type
- 1"
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'visited'
  values:
  - {key: 'wp', value: 'wp$i'}
  function_value: 0.0"
done;

rosservice call /rosplan_knowledge_base/update_array "
$param_type
$param"

# automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
echo "Calling problem generator.";
rosservice call /rosplan_problem_interface/problem_generation_server;

# make plan (e.g. call popf to create solution)
echo "Calling planner interface.";
rosservice call /rosplan_planner_interface/planning_server;

# parse plan (parse console output and extract actions and params, e.g. create esterel graph)
echo "Calling plan parser.";
rosservice call /rosplan_parsing_interface/parse_plan;

# dispatch (execute) plan. (send actions one by one to their respective interface and wait for them to finish)
echo "Calling plan dispatcher.";
rosservice call /rosplan_plan_dispatcher/dispatch_plan;

echo "Finished!";
