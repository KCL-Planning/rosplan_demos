#!/usr/bin/env python

import rospy, tf, actionlib
import actionlib_msgs.msg

from rosplan_planning_system.ActionInterfacePy.RPActionInterface import RPActionInterface
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RPMoveBasePy(RPActionInterface):
    def __init__(self):
        # call parent constructor
        RPActionInterface.__init__(self)
        # setup a move base clear costmap client (to be able to send clear costmap requests later on)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_srv_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # get waypoints reference frame from param server
        self.waypoint_frameid = rospy.get_param('~waypoint_frameid', 'map')
        self.wp_namespace = rospy.get_param('~wp_namespace', '/rosplan_demo_waypoints/wp')
        # create move base action client
        actionserver = rospy.get_param('~action_server', '/move_base')
        self.action_client = actionlib.SimpleActionClient(actionserver, MoveBaseAction)

    def wpIDtoPoseStamped(self, wpID):
        result = None
        if rospy.has_param(self.wp_namespace + '/' + wpID):
            wp = rospy.get_param(self.wp_namespace + '/' + wpID)
            if wp:
                if len(wp) == 3:
                    result = PoseStamped()
                    result.header.frame_id = self.waypoint_frameid
                    # result.header.stamp = rospy.Time.now()
                    result.pose.position.x = wp[0]
                    result.pose.position.y = wp[1]
                    result.pose.position.z = 0.0

                    q = tf.transformations.quaternion_from_euler(0, 0, wp[2])
                    result.pose.orientation.x = q[0]
                    result.pose.orientation.y = q[1]
                    result.pose.orientation.z = q[2]
                    result.pose.orientation.w = q[3]
                else:
                    rospy.logerr('wp size must be equal to 3 : (x, y, and theta)')
        return result

    def concreteCallback(self, msg):
        # get waypoint ID from action dispatch msg
        wpID = ''
        found = False
        # iterating over parameters (e.g. kenny, wp0, wp1)
        for i in range(0, len(msg.parameters)):
            # check their keys
            if('to' in msg.parameters[i].key) or ('w1' in msg.parameters[i].key):
                # wp id found in msg params
                wpID = msg.parameters[i].value
                found = True
        if not found:
            rospy.loginfo('KCL: ({}) aborting action dispatch PDDL action missing required parameter ?to'.format(self.params.name))
            return False

        # get waypoint coordinates from its ID via query to parameter server
        pose = PoseStamped()
        pose = self.wpIDtoPoseStamped(wpID)
        if not pose:
            rospy.logerr('Waypoint not found in parameter server')
            return False

        rospy.loginfo('KCL: ({}) waiting for move_base action server to start'.format(self.params.name))
        self.action_client.wait_for_server()

        # dispatch MoveBase action
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.action_client.send_goal(goal)

        finished_before_timeout = self.action_client.wait_for_result()
        if finished_before_timeout:

            state = self.action_client.get_state()
            rospy.loginfo('KCL: ({}) action finished: {}'.format(self.params.name, state))

            if state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                # publish feedback (achieved)
                return True
            else:
                # clear costmaps
                self.clear_costmaps_srv_client(Empty())
                # publish feedback (failed)
                return False
        else:
            # timed out (failed)
            self.action_client.cancelAllGoals()
            rospy.loginfo('KCL: ({}) action timed out'.format(self.params.name))
            return False

def main():
    rospy.init_node('rosplan_interface_movebase', anonymous=False)
    rpmb = RPMoveBasePy()
    rpmb.runActionInterface()
