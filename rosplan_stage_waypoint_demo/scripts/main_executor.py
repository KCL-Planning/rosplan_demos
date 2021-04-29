#!/usr/bin/env python
import rospkg
import rospy
import sys
import time
import os
import time

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
wait_for_rviz = rospy.get_param('~wait_for_rviz', False)
max_sample_size = rospy.get_param('~max_sample_size', 70)
max_prm_size = rospy.get_param('~max_prm_size', 1000)
approach = rospy.get_param('~approach', 0)
domain_path = rospy.get_param('~domain_path', "")
problem_path = rospy.get_param('~autom_gen_problem_path', "")
data_path = rospy.get_param('~data_path', "")
initial_state = rospy.get_param('~initial_state', "param_not_set")
results_path = rospy.get_param('~results_path', "results.csv")
planner_command = rospy.get_param('~planner_command', "")

# wait for services
rospy.wait_for_service('/rosplan_roadmap_server/create_prm')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server_params')
rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
if approach == 0:
    rospy.wait_for_service('/waypoint_sampler/sample_waypoints')

vispub = rospy.Publisher('/update_visibility', String, queue_size=10)

plan = ""
plan_recieved = False
planning_time = 0
def plan_callback(msg):
    global plan, plan_recieved
    plan = msg.data
    plan_recieved = True

sub_once = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, plan_callback)

def make_prm(size):
    # generate dense PRM
    rospy.loginfo("KCL: (%s) Creating PRM of size %i" % (rospy.get_name(), size))
    prm = rospy.ServiceProxy('/rosplan_roadmap_server/create_prm', CreatePRM)        
    if not prm(size,0.8,1.6,2.0,50,200000):
        rospy.logerr("KCL: (%s) No PRM was made" % rospy.get_name())

def wait_for_sensing():

    rospy.loginfo("KCL: (%s) Triggering the visibility update" % rospy.get_name())
    vispub.publish("update_1")
    vispub.publish("update_2")

    # wait for the sensing interface to catch up
    rospy.loginfo("KCL: (%s) Waiting for visibility to be added to KB" % rospy.get_name())
    propcount = 0
    start = time.time()
    while propcount < 1 and (time.time() - start < 20):
        vis = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
        vis_response = vis("doughnut_visible_from")
        if not vis_response:
            rospy.logerr("KCL: (%s) Failed to call the KB" % rospy.get_name())
            quit()
        propcount = len(vis_response.attributes)
        rospy.sleep(0.5)

    wp_id = "NO_WAYPOINT"
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

def generate_problem_and_plan():
    global planning_time
    rospy.loginfo("KCL: (%s) Calling problem generation" % rospy.get_name())
    pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
    if not pg():
        rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

    rospy.loginfo("KCL: (%s) Calling planner" % rospy.get_name())
    pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server_params', PlanningService)

    start = time.time()
    pi_response = pi(domain_path, problem_path, data_path, planner_command, True)
    end = time.time()
    planning_time = (end - start)

    if not pi_response:
        rospy.logerr("KCL: (%s) No response from the planning server." % rospy.get_name())
        return False
    if not pi_response.plan_found:
        rospy.loginfo("KCL: (%s) No plan could be found." % rospy.get_name())
        return False
    else:
        rospy.loginfo("KCL: (%s) Plan was found." % rospy.get_name())
        return True

def plan_cost():
    global planning_time, plan

    action_list = plan.splitlines()

    # get plan duration
    t1 = action_list[len(action_list)-1].split(":")[0]
    t2 = action_list[len(action_list)-1].split("[")[1].split("]")[0]
    plan_duration = float(t1) + float(t2)

    # get the cost parameters for waypoints
    wp_costs = rospy.get_param('/task_planning_waypoints_pref')
    rospy.loginfo(wp_costs)

    # get distance travelled
    total_distance = 0
    total_cost = 0
    current_wp_cost = 0
    old_time = 0
    for a in action_list:
        if 'goto_waypoint' in a:

            ta = a.split("[")[1].split("]")[0]
            total_distance += float(ta)

            rospy.loginfo('======================')
            rospy.loginfo(a)
            new_time = a.split(" ")[0][0:len(a.split(" ")[0])-1]
            rospy.loginfo(f'new time stamp: {new_time}')
            time_at_wp = float(new_time) - old_time
            old_time = float(new_time)
            incurred_cost = time_at_wp * current_wp_cost
            total_cost = total_cost + incurred_cost
            rospy.loginfo(f'increasing cost by: {str(time_at_wp)} * {str(current_wp_cost)} = {str(incurred_cost)}')

            rospy.loginfo('----------------------')
            # 0:dispatch_time 1:action_name 2:vehicle 3:from 4:to) 5:duration
            ta = a.split(" ")[4][0:len(a.split(" ")[4])-1]
            rospy.loginfo(f'new wp: {ta}')
            current_wp_cost = 100 - wp_costs[ta]
            rospy.loginfo(f'new wp cost: {str(current_wp_cost)}')


    rospy.loginfo('======================')
    rospy.loginfo(f'final time: {str(plan_duration)}')
    time_at_wp = plan_duration - old_time
    incurred_cost = time_at_wp * current_wp_cost
    total_cost = total_cost + incurred_cost
    rospy.loginfo("increasing cost by: " + str(time_at_wp) + " * " + str(current_wp_cost) + " = " +  str(incurred_cost)
    rospy.loginfo(f'increasing cost by: {str(time_at_wp)} * {str(current_wp_cost)} = {str(incurred_cost)}')

    try:
        f = open(results_path, "a")
        f.write(str(approach)+","+os.path.basename(initial_state)+","+str(plan_duration)+","+str(total_distance)+","+str(planning_time)+","+str(total_cost)+"\n")
    except:
        rospy.logerr("KCL: (%s) Error writing to results file." % rospy.get_name())     
    finally:
        f.close()

def plan_failed():
    try:
        f = open(results_path, "a")
        f.write(str(approach)+","+os.path.basename(initial_state)+",-1,-1,-1\n")
    except:
        rospy.logerr("KCL: (%s) Error writing to results file." % rospy.get_name())     
    finally:
        f.close()

### EXPERIMENT ###
try:
    rospy.sleep(1)
    ### PRM APPROACH ###
    if approach == 1:
        sample_count = 3
        while sample_count < max_prm_size:
            make_prm(sample_count)
            wait_for_sensing()

            plan_found = generate_problem_and_plan()
            if not plan_found:
                if planning_time>=10.00:
                    # timeout, stop here
                    sample_count = max_prm_size
                else:
                    # not a timeout, try again
                    sample_count += 1
            else:
                # Wait for plan
                while not rospy.is_shutdown() and not plan_recieved:
                    rospy.loginfo("KCL: (%s) Plan not received, waiting..." % rospy.get_name())
                    rospy.sleep(0.5)
                plan_cost()
                break
        if not plan_found:
            plan_failed()

    ### SAMPLING APPROACH ###
    resamples = 0
    if approach == 0 or approach == 2:

        make_prm(max_prm_size)

        sample_count = 12
        goal_achieved = False
        if approach==2:
            max_sample_size = sample_count + 1

        while sample_count < max_sample_size:

            rospy.loginfo("KCL: (%s) Sampling %i waypoints" % (rospy.get_name(), sample_count))
            smp = rospy.ServiceProxy('/waypoint_sampler/sample_waypoints', SetInt)        
            if not smp(sample_count):
                rospy.logerr("KCL: (%s) No sample was made" % rospy.get_name())

            wait_for_sensing()

            plan = ""
            plan_recieved = False
            plan_found = generate_problem_and_plan()

            if not plan_found:
                if planning_time>=10.00 and approach==0:
                    if resamples>4:
                        break
                    # timeout, decrease sample size
                    sample_count -= 4
                    if sample_count < 8:
                        sample_count = 8
                    resamples += 1
                else:
                    sample_count += 2
            else:
                # Wait for plan
                while not rospy.is_shutdown() and not plan_recieved:
                    rospy.loginfo("KCL: (%s) Plan not received, waiting..." % rospy.get_name())
                    rospy.sleep(0.5)
                plan_cost()
                break
        if not plan_found:
            plan_failed()

except rospy.ServiceException as e:
    rospy.logerr("KCL: (%s) Service call failed: %s" % (rospy.get_name(), e))
