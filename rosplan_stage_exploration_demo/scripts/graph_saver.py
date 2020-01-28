#!/usr/bin/env python
import rospkg
import rospy
from std_msgs.msg import String

def callback_plan(data):
    with open(data_path, "w") as f1:
        try:
            f1.write(data.data)
        except:
            rospy.logerr("KCL: (%s) Error writing to plan graph to file." % rospy.get_name())

def listener():
    rospy.Subscriber("/rosplan_plan_dispatcher/plan_graph", String, callback_plan)
    rospy.spin()

rospy.init_node('graph_saver', anonymous=True)
data_path = rospy.get_param('~data_path', "")
listener()
