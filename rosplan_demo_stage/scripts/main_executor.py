#!/usr/bin/env python
import rospkg
import rospy
import sys
import time
import os

from std_srvs.srv import Empty, EmptyResponse
from rosplan_knowledge_msgs.srv import SetInt, GetAttributeService
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse

# get path of pkg
rospack = rospkg.RosPack()
rospy.init_node("coordinator")

problem_path = rospy.get_param('/coordinator/problem_path', "problem.pddl")
output_directory = rospy.get_param('/coordinator/output_directory', 0.0)

rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
rospy.wait_for_service('/waypoint_sampler/sample_waypoints')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server')
rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')

try:

    prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
    if not prm(1000,0.8,1.6,2.0,50,200000):
        print('NO PRM')

    # add goal to visit the doughnuts

    sample_count = 1
    goal_achieved = False
    while sample_count<20:

        smp = rospy.ServiceProxy('/waypoint_sampler/sample_waypoints', SetInt)        
        if not smp(sample_count):
            print('NO SAMPLE')

        # wait for the sensing interface to catch up
        propcount = 0
        while propcount < 1:
            vis = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
            vis_response = vis("doughnut_visible_from")
            if not vis_response:
                print('FAILED TO CALL KB')
                quit()
            propcount = len(vis_response.attributes)
            print('WAITING FOR SENSING')
            rospy.sleep(0.5)

        pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        if not pg():
            print('NO PROBLEM')

        pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        if not pi():
            print('NO PLAN')
            break

        sample_count+=1

except rospy.ServiceException, e:
    print "Service call failed: %s"%e
    replans += 1
