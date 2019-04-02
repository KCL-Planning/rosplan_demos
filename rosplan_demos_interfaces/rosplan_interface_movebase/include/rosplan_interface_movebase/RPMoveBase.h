#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPMoveBase class.
 * RPMoveBase is used to connect ROSPlan to the MoveBase library.
 * PDDL "goto_waypoint" actions become "move_base_msgs::MoveBase" actions.
 * Waypoint goals are fetched by name from the parameter server.
 */

namespace KCL_rosplan {

    class RPMoveBase: public RPActionInterface
    {

    private:

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
        ros::ServiceClient clear_costmaps_client;

    public:

        /* constructor */
        RPMoveBase(ros::NodeHandle &nh, std::string &actionserver);

        /* input waypoint id, output a pose. Information is fetched from parameter server */
        bool wpIDtoPoseStamped(std::string wpID, geometry_msgs::PoseStamped &result);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

        // waypoints reference frame
        std::string waypoint_frameid_;
    };
}
#endif
