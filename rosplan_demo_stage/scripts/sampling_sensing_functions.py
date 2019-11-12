#!/usr/bin/env python
import numpy as np
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import GetInstanceService
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from math import sqrt

####################
# SETTING robot_at #
####################

def robot_at(msg, params): # Idea: Params are the instances of all the parameters in a dictionary and the messaestoreproxy
    assert(msg.header.frame_id == "map")
    assert(len(params) == 2)
    ret_value = []
    attributes = get_kb_attribute("robot_at")
    curr_wp = ''
    # Find current robot_location
    for a in attributes:
        if not a.is_negative:
            curr_wp = a.values[1].value
            rospy.loginfo("KCL: (%s) Robot is at $s" % rospy.get_name(), curr_wp)
            break

    for robot in params[0]:
        distance = float('inf')
        closest_wp = ''
        for wp in params[1]:
            if rospy.has_param("/task_planning_waypoints/"+wp):
                pose = rospy.get_param("/task_planning_waypoints/"+wp)
                if len(pose) > 0:
                    x = pose[0] - msg.pose.pose.position.x
                    y = pose[1] - msg.pose.pose.position.y
                    d = sqrt(x**2 + y**2)
                    if d < distance:
                        closest_wp = wp
                        distance = d
        if curr_wp != closest_wp:
            if curr_wp != '':
                ret_value.append((robot + ':' + curr_wp, False)) # Set current waypoint to false
            ret_value.append((robot + ':' + closest_wp, True))  # Set new wp to true
    return ret_value

#################################
# SETTING doughnut_visible_from #
#################################

_static_map = None

global set_map
def set_map(msg):
    global _static_map
    rospy.loginfo("KCL: (%s) Static map received!" % rospy.get_name())
    _static_map = msg

def doughnut_visible_from(res, params):

    global _static_map

    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    update_kb = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)

    # Wait for map
    sub_once = rospy.Subscriber("map", OccupancyGrid, set_map)
    while not rospy.is_shutdown() and not _static_map:
        rospy.loginfo("KCL: (%s) Static map not received, waiting..." % rospy.get_name())
        rospy.sleep(0.5)
    sub_once.unregister()

    assert(len(params) == 2)
    ret_value = []
    skip_check = False

    # load and check KB for objects
    doughnuts = []
    if rospy.has_param('~doughnuts'):
        doughnuts = rospy.get_param('~doughnuts')
    if len(params[1]) != len(doughnuts):
        del params[1][:]
        # objects not properly loaded yet, add to KB
        rospy.loginfo("KCL: (%s) Objects not loaded into KB yet, skipping visibility check" % rospy.get_name())
        try:
            kus = KnowledgeUpdateServiceArrayRequest()
            for d in doughnuts:
                ki = KnowledgeItem()
                ki.instance_type = d['type']
                ki.instance_name = d['name']
                ki.knowledge_type = ki.INSTANCE
                kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                kus.knowledge.append(ki)
                params[1].append(d['name'])
            res = update_kb(kus)
        except rospy.ServiceException, e:
            rospy.logerr("KCL (%s) Failed to update knowledge base: %s" % rospy.get_name(), e.message)
        skip_check = True

    # load and check KB for waypoints
    waypoints = []
    if rospy.has_param("/task_planning_waypoints"):
        waypoints = rospy.get_param('/task_planning_waypoints')
    if len(params[0]) != len(waypoints):
        del params[0][:]
        # waypoints not properly loaded yet, add to KB
        rospy.loginfo("KCL: (%s) Waypoints not loaded into KB yet, skipping visibility check" % rospy.get_name())
        try:
            kus = KnowledgeUpdateServiceArrayRequest()
            for wp in waypoints:
                ki = KnowledgeItem()
                ki.instance_type = "waypoint"
                ki.instance_name = wp
                ki.knowledge_type = ki.INSTANCE
                kus.update_type += np.array(kus.ADD_KNOWLEDGE).tostring()
                kus.knowledge.append(ki)
                params[0].append(wp)
            res = update_kb(kus)
        except rospy.ServiceException, e:
            rospy.logerr("KCL (%s) Failed to update knowledge base: %s" % rospy.get_name(), e.message)
        skip_check = True

    if skip_check:
        return ret_value

    attributes = get_kb_attribute("doughnut_visible_from")

    topic = 'doughnut_visible_from'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)
    markerArray = MarkerArray() 

    marker = Marker()
    marker.id = 0
    marker.header.frame_id = "map"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    # check for each doughnut
    count = 0
    for ped in params[1]:

        ped_req = doughnuts[count]
        count = count + 1

        # check from each waypoint
        for wp in params[0]:

            visible = False
            for a in attributes:
                if not a.is_negative and wp == a.values[0].value and ped == a.values[1].value:
                    visible = True

            # fetch the doughnut requirements
            if rospy.has_param("/task_planning_waypoints/"+wp):

                pose = []
                try:
                    pose = rospy.get_param("/task_planning_waypoints/"+wp)
                except KeyError, e:
                    return ret_value

                if len(pose) > 0:

                    (xA, yA) = (pose[0], pose[1])
                    (xB, yB) = (ped_req['x'], ped_req['y'])
                    (dx, dy) = (xB - xA, yB - yA)

                    # the perfect distance from which to view is ped_req[2]
                    d = sqrt(dx**2 + dy**2)
                    d = abs(d - ped_req['radius'])
                    inDistance = False
                    collision = False
                    if d < 2*ped_req['std_dev']:

                        inDistance = True
                        sx = _static_map.info.resolution * dx/(abs(dx)+abs(dy))
                        sy = _static_map.info.resolution * dy/(abs(dx)+abs(dy))
                        signsx = (sx > 0) - (sx < 0)
                        signsy = (sy > 0) - (sy < 0)

                        while (not collision and (sx == 0 or xA*signsx < xB*signsx) and (sy == 0 or yA*signsy < yB*signsy)):

                            gx = int((xA - _static_map.info.origin.position.x)/_static_map.info.resolution)
                            gy = int((yA - _static_map.info.origin.position.y)/_static_map.info.resolution)
                            if _static_map.data[gx + gy*_static_map.info.width] > 0.12:
                                collision = True
                            xA += sx
                            yA += sy

                    if not collision and inDistance:
                        ret_value.append((wp + ':' + ped, True))
                        visible = True
                    elif (not inDistance or collision) and visible:
                        ret_value.append((wp + ':' + ped, False))
                    # for debugging
                    if visible:
                        marker.points.append(Point(pose[0],pose[1],0))
                        marker.points.append(Point(ped_req['x'],ped_req['y'],0))
                        markerArray.markers.append(marker)

    # Publish the MarkerArray
    publisher.publish(markerArray)

    return ret_value
