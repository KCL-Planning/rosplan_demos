#!/usr/bin/env python
import rospkg
import rospy
import sys
import time
import os

from std_srvs.srv import Empty, EmptyResponse
from rosplan_knowledge_msgs.srv import *
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from diagnostic_msgs.msg import KeyValue

# get path of pkg
rospack = rospkg.RosPack()
rospy.init_node("coordinator")

# load parameters
wait_for_rviz = rospy.get_param('~wait_for_rviz', False)
max_sample_size = rospy.get_param('~max_sample_size', 10)
max_prm_size = rospy.get_param('~max_prm_size', 1000)
approach = rospy.get_param('~approach', 0)
domain_path = rospy.get_param('~domain_path', "")
problem_path = rospy.get_param('~autom_gen_problem_path', "")
data_path = rospy.get_param('~data_path', "")
planner_command = rospy.get_param('~planner_command', "")

# wait for services
rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server_params')
rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
if approach == 0:
    rospy.wait_for_service('/waypoint_sampler/sample_waypoints')

# begin experiment
try:

    # generate dense PRM
    rospy.loginfo("KCL: (%s) Creating PRM of size %i" % (rospy.get_name(), max_prm_size))
    prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
    if not prm(max_prm_size,0.8,1.6,2.0,50,200000):
        rospy.logerr("KCL: (%s) No PRM was made" % rospy.get_name())

    ### SAMPLING APPROACH ###
    if approach == 0 or approach == 2:

        sample_count = 1
        goal_achieved = False

        if approach==2:
            max_sample_size = sample_count + 1

        while sample_count < max_sample_size:

            rospy.loginfo("KCL: (%s) Sampling %i waypoints" % (rospy.get_name(), sample_count))
            smp = rospy.ServiceProxy('/waypoint_sampler/sample_waypoints', SetInt)        
            if not smp(sample_count):
                rospy.logerr("KCL: (%s) No sample was made" % rospy.get_name())

            # wait for the sensing interface to catch up
            rospy.loginfo("KCL: (%s) Waiting for visibility to be added to KB" % rospy.get_name())
            propcount = 0
            while propcount < 1:
                vis = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
                vis_response = vis("doughnut_visible_from")
                if not vis_response:
                    rospy.logerr("KCL: (%s) Failed to call the KB" % rospy.get_name())
                    quit()
                propcount = len(vis_response.attributes)
                rospy.sleep(0.5)

            wp_id = "GERARDDDD!!"
            inst = rospy.ServiceProxy('/rosplan_knowledge_base/state/instances', GetInstanceService)
            inst_response = inst("waypoint")
            if len(inst_response.instances):
                wp_id = inst_response.instances[0]

            kus = KnowledgeUpdateServiceRequest()
            kus.knowledge.knowledge_type = 1
            kus.knowledge.attribute_name = 'robot_at'
            kv = KeyValue()
            kv.key = 'r'
            kv.value = 'kenny'
            kus.knowledge.values.append(kv)
            kv = KeyValue()
            kv.key = 'wp'
            kv.value = wp_id
            kus.knowledge.values.append(kv)
            kuc = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)        
            if not kuc(kus):
                rospy.logerr("KCL: (%s) Robot at was not added!" % rospy.get_name())
                break

            rospy.loginfo("KCL: (%s) Calling problem generation" % rospy.get_name())
            pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
            if not pg():
                rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

            rospy.loginfo("KCL: (%s) Calling planner" % rospy.get_name())
            pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server_params', PlanningService)
            pi_response = pi(domain_path, problem_path, data_path, planner_command, True)
            if not pi_response:
                rospy.logerr("KCL: (%s) No response from the planning server." % rospy.get_name())
                break
            if not pi_response.plan_found:
                rospy.loginfo("KCL: (%s) No plan could be found." % rospy.get_name())
                sample_count += 1
            else:
                rospy.loginfo("KCL: (%s) Plan was found." % rospy.get_name())
                break

except rospy.ServiceException, e:
    rospy.logerr("KCL: (%s) Service call failed: %s" % (rospy.get_name(), e))
