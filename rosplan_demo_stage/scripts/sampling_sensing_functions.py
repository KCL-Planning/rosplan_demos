#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from rosplan_knowledge_msgs.srv import GetInstanceService
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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
# SETTING pedestal_visible_from #
#################################

def pedestal_visible_from(res, params):

    assert(len(params) == 2)
    ret_value = []
    attributes = get_kb_attribute("pedestal_visible_from")

    doughnuts = []
    if rospy.has_param('~doughnuts'):
        doughnuts = rospy.get_param('~doughnuts')

    assert(len(params[1])==len(doughnuts))

    topic = 'pedestal_visible_from'
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
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    # check for each pedestal
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

            # fetch the pedestal requirements
            if rospy.has_param("/task_planning_waypoints/"+wp):
                pose = rospy.get_param("/task_planning_waypoints/"+wp)
                if len(pose) > 0:
                    x = pose[0] - ped_req[0]
                    y = pose[1] - ped_req[1]
                    d = sqrt(x**2 + y**2)
                    # the perfect distance from which to view is ped_req[2]
                    d = abs(d - ped_req[2])
                    if d < 2*ped_req[3] and not visible:
                        ret_value.append((wp + ':' + ped, True))
                        visible = True
                    elif not d < 2*ped_req[3] and visible:
                        ret_value.append((wp + ':' + ped, False))
                    # for debugging
                    if visible:
                        marker.points.append(Point(pose[0],pose[1],0))
                        marker.points.append(Point(ped_req[0],ped_req[1],0))
                        markerArray.markers.append(marker)

    # Publish the MarkerArray
    publisher.publish(markerArray)

    return ret_value
