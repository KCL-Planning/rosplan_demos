#!/usr/bin/env python
import rospkg
import rospy
import sys
import time
import os
import time
import random

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from rosplan_knowledge_msgs.srv import *
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from diagnostic_msgs.msg import KeyValue

############
# THE REST #
############

# get path of pkg
rospack = rospkg.RosPack()
rospy.init_node("coordinator")

# load parameters
max_prm_size = rospy.get_param('~max_prm_size', 1000)
planner_command = rospy.get_param('~planner_command', "")
domain_path = rospy.get_param('~domain_path', "")
problem_path = rospy.get_param('~problem_path', "")
data_path = rospy.get_param('~data_path', "")

# wait for services
rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server_params')
rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')

def make_prm(size):
    # generate dense PRM
    rospy.loginfo("KCL: (%s) Creating PRM of size %i" % (rospy.get_name(), size))
    prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
    if not prm(size,0.8,1.6,2.0,65,200000):
        rospy.logerr("KCL: (%s) No PRM was made" % rospy.get_name())

def generate_problem_and_plan():

    rospy.loginfo("KCL: (%s) Calling problem generation" % rospy.get_name())
    pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    if not pg():
        rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

    rospy.loginfo("KCL: (%s) Calling planner" % rospy.get_name())
    pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server_params', PlanningService)
    pi_response = pi(domain_path, problem_path, data_path, planner_command, True)

    if not pi_response:
        rospy.logerr("KCL: (%s) No response from the planning server." % rospy.get_name())
        return False
    if not pi_response.plan_found:
        rospy.loginfo("KCL: (%s) No plan could be found." % rospy.get_name())
        return False
    else:
        rospy.loginfo("KCL: (%s) Plan was found." % rospy.get_name())
        return True

def execute_plan():

    rospy.loginfo("KCL: (%s) Calling plan parser" % rospy.get_name())
    pp = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
    if not pp():
        rospy.logerr("KCL: (%s) The plan was not parsed!" % rospy.get_name())
        return

    rospy.sleep(3)

    rospy.loginfo("KCL: (%s) Calling plan execution" % rospy.get_name())
    pd = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    pd_response = pd()

    if not pd_response:
        rospy.logerr("KCL: (%s) No response from the dispatch server." % rospy.get_name())
        return False
    if not pd_response.goal_achieved:
        rospy.loginfo("KCL: (%s) The execution was not successful." % rospy.get_name())
        return False
    else:
        rospy.loginfo("KCL: (%s) Plan was executed." % rospy.get_name())
        return True

### execute visit all ####
try:
    rospy.sleep(3)
    make_prm(max_prm_size)

    # wait for odom to publish to sensing interface
    sps = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)    
    count = 0
    while count<1:
        rospy.sleep(1)
        gas = GetAttributeServiceRequest()    
        gas.predicate_name = 'robot_at'
        facts = sps(gas)
        if not facts:
            rospy.logwarn("KCL: (%s) Proposition service not available." % rospy.get_name())
        count = 0
        for k in facts.attributes:
            if not k.is_negative:
                count = count + 1

    # add goals to the KB
    for i in range(3):
        wp_goal = random.randint(0,max_prm_size-1)
        kus = KnowledgeUpdateServiceRequest()
        kus.update_type = 1
        kus.knowledge.knowledge_type = 1
        kus.knowledge.attribute_name = 'visited'
        kv = KeyValue()
        kv.key = 'wp'
        kv.value = 'wp'+str(wp_goal)
        kus.knowledge.values.append(kv)
        kuc = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        if not kuc(kus):
            rospy.logerr("KCL: (%s) Goal was not added!" % rospy.get_name())

    plan_found = generate_problem_and_plan()
    rospy.sleep(1)

    if plan_found:
        execute_plan()

except rospy.ServiceException as e:
    rospy.logerr("KCL: (%s) Service call failed: %s" % (rospy.get_name(), e))
